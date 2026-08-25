"""Measure stable ROS command and device telemetry rates on one hand."""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable, Sequence
from itertools import pairwise

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from linkerbot_hand import SensorType, get_model_capabilities
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray

from linkerbot_hand_ros.adapter import validate_command
from linkerbot_hand_ros.benchmark import (
    BenchmarkMeasurements,
    BenchmarkResult,
    DiagnosticObservation,
    StateEvent,
    evaluate_measurements,
)
from linkerbot_hand_ros.client import HandCommandClient
from linkerbot_hand_ros.examples._command_cli import (
    DEFAULT_DRIVER_NAME,
    resolve_driver_name,
    spin_until_ready,
)
from linkerbot_hand_ros.qos import METADATA_QOS, STATE_QOS

DEFAULT_RATES = (50.0, 60.0, 80.0, 100.0)
CONTROL_MODES = ("control", "angles", "torques", "mixed")


def _parse_args(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="测量灵巧手 50-100 Hz 命令和真实设备采样频率"
    )
    parser.add_argument("--driver-name", default=DEFAULT_DRIVER_NAME)
    parser.add_argument(
        "--mode",
        choices=CONTROL_MODES,
        default="control",
        help="control=设置一次力矩限制后连续发送角度",
    )
    parser.add_argument(
        "--feedback",
        choices=("angles-and-load", "angles"),
        default="angles-and-load",
        help="同时评估角度与力矩/电流反馈，或只评估角度反馈",
    )
    parser.add_argument(
        "--rates",
        type=float,
        nargs="+",
        default=list(DEFAULT_RATES),
        help="按升序测试的总命令频率",
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--drain-time", type=float, default=1.0)
    parser.add_argument("--startup-timeout", type=float, default=10.0)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument(
        "--continue-after-failure",
        action="store_true",
        help="某一频率失败后仍继续测试更高频率",
    )
    parser.add_argument(
        "--no-configure-driver",
        action="store_true",
        help="不自动同步 polling、状态发布和诊断频率",
    )
    torque_group = parser.add_mutually_exclusive_group()
    torque_group.add_argument(
        "--torque-value",
        type=float,
        help="所有关节使用同一力矩或电流限制",
    )
    torque_group.add_argument(
        "--torque-values",
        type=float,
        nargs="+",
        help="按 joint_names 顺序设置逐关节限制",
    )
    parser.add_argument(
        "--angle-values",
        type=float,
        nargs="+",
        help="角度基准；省略时保持首次收到的当前姿态",
    )
    parser.add_argument(
        "--angle-amplitude",
        type=float,
        default=0.0,
        help="单关节正弦扰动幅度，默认 0 表示不主动运动",
    )
    parser.add_argument("--angle-period", type=float, default=2.0)
    parser.add_argument("--joint-index", type=int, default=0)
    return parser.parse_args(args)


def _validate_args(parsed: argparse.Namespace) -> None:
    rates = parsed.rates
    if not rates or any(not math.isfinite(rate) or rate <= 0.0 for rate in rates):
        raise ValueError("--rates 必须是有限正数")
    if any(later <= earlier for earlier, later in pairwise(rates)):
        raise ValueError("--rates 必须严格递增且不能重复")
    for name in ("duration", "warmup", "drain_time", "startup_timeout"):
        value = getattr(parsed, name)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"--{name.replace('_', '-')} 必须是有限正数")
    if parsed.repetitions <= 0:
        raise ValueError("--repetitions 必须大于 0")
    if parsed.joint_index < 0:
        raise ValueError("--joint-index 不能为负数")
    if not math.isfinite(parsed.angle_amplitude) or parsed.angle_amplitude < 0.0:
        raise ValueError("--angle-amplitude 必须是有限非负数")
    if not math.isfinite(parsed.angle_period) or parsed.angle_period <= 0.0:
        raise ValueError("--angle-period 必须是有限正数")
    if parsed.mode in {"control", "torques", "mixed"} and (
        parsed.torque_value is None and parsed.torque_values is None
    ):
        raise ValueError(
            f"{parsed.mode} 模式必须显式提供 --torque-value 或 --torque-values"
        )


class FrequencyBenchmarkNode(Node):
    """Persistent publishers and subscribers used by the benchmark loop."""

    def __init__(self, driver_name: str) -> None:
        super().__init__("linkerbot_hand_frequency_benchmark")
        self.driver_name = driver_name.rstrip("/")
        self.command_client = HandCommandClient(self, self.driver_name)
        self.parameter_client = AsyncParameterClient(self, self.driver_name)
        self.latest_states: dict[str, tuple[float, ...]] = {}
        self.seen_sensors: set[str] = set()
        self.state_events: list[StateEvent] = []
        self.accepted_events: list[tuple[str, float]] = []
        self.diagnostic_observations: list[DiagnosticObservation] = []

        for sensor in ("angles", "torques", "current"):
            self.create_subscription(
                Float64MultiArray,
                f"{self.driver_name}/state/{sensor}",
                lambda message, name=sensor: self._on_state(name, message),
                STATE_QOS,
            )
        for command in ("angles", "torques"):
            self.create_subscription(
                Float64MultiArray,
                f"{self.driver_name}/command/accepted/{command}",
                lambda _message, name=command: self._on_accepted(name),
                METADATA_QOS,
            )
        self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self._on_diagnostics,
            STATE_QOS,
        )

    @property
    def latest_diagnostic(self) -> DiagnosticObservation | None:
        """Return the most recently received status for the target driver."""
        if not self.diagnostic_observations:
            return None
        return self.diagnostic_observations[-1]

    def _on_state(self, sensor: str, message: Float64MultiArray) -> None:
        observed_at = time.monotonic()
        values = tuple(float(value) for value in message.data)
        joint_count = len(self.command_client.joint_names)
        valid = (
            joint_count > 0
            and len(values) == joint_count
            and all(math.isfinite(value) for value in values)
        )
        self.seen_sensors.add(sensor)
        if valid:
            self.latest_states[sensor] = values
        else:
            self.latest_states.pop(sensor, None)
        self.state_events.append(StateEvent(sensor, observed_at, valid))

    def _on_accepted(self, command: str) -> None:
        self.accepted_events.append((command, time.monotonic()))

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        expected_name = f"{self.driver_name}: hand_driver"
        for status in message.status:
            if status.name != expected_name:
                continue
            self.diagnostic_observations.append(
                DiagnosticObservation(
                    observed_at=time.monotonic(),
                    values={item.key: item.value for item in status.values},
                )
            )

    def configure_rates(
        self,
        rate_hz: float,
        secondary_sensor: str | None,
        timeout_sec: float,
    ) -> None:
        """Set runtime driver rates and wait for the atomic response."""
        if not self.parameter_client.wait_for_services(timeout_sec=timeout_sec):
            raise RuntimeError(f"参数服务未就绪: {self.driver_name}")
        parameters = [
            Parameter("angles_hz", value=rate_hz),
            Parameter("publish_rate_hz", value=rate_hz),
            Parameter("diagnostic_rate", value=min(rate_hz, 20.0)),
        ]
        if secondary_sensor is not None:
            parameters.append(Parameter(f"{secondary_sensor}_hz", value=rate_hz))
        future = self.parameter_client.set_parameters_atomically(parameters)
        _spin_until_future(self, future, timeout_sec)
        response = future.result()
        if response is None or not response.result.successful:
            reason = "无响应" if response is None else response.result.reason
            raise RuntimeError(f"驱动频率配置失败: {reason}")

    def destroy_benchmark(self) -> None:
        """Release the command client before destroying this node."""
        self.command_client.destroy()


def _spin_until_future(node: Node, future: object, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError("等待 ROS 服务响应超时")


def _spin_until(
    node: Node,
    predicate: Callable[[], bool],
    timeout_sec: float,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=min(0.05, deadline - time.monotonic()))
    return predicate()


def _secondary_sensor(model: str) -> str:
    capabilities = get_model_capabilities(model)
    if capabilities.supports_sensor(SensorType.TORQUES):
        return "torques"
    if capabilities.supports_sensor(SensorType.CURRENT):
        return "current"
    raise RuntimeError(f"{model} 不支持力矩或电流反馈")


def _resolve_targets(
    parsed: argparse.Namespace,
    node: FrequencyBenchmarkNode,
) -> tuple[tuple[float, ...] | None, tuple[float, ...] | None]:
    model = node.command_client.model
    if model is None:
        raise RuntimeError("未收到驱动型号元数据")
    joint_count = len(node.command_client.joint_names)

    angle_target: tuple[float, ...] | None = None
    if parsed.mode in {"control", "angles", "mixed"}:
        values = parsed.angle_values or node.latest_states.get("angles")
        if values is None:
            raise RuntimeError(
                "没有完整有限的当前角度；请检查轮询或显式传 --angle-values"
            )
        angle_target = tuple(
            float(value) for value in validate_command(model, values, "angles")
        )
        if parsed.joint_index >= joint_count:
            raise ValueError(f"--joint-index 必须小于关节数量 {joint_count}")

    torque_target: tuple[float, ...] | None = None
    if parsed.mode in {"control", "torques", "mixed"}:
        values = parsed.torque_values
        if values is None:
            values = [parsed.torque_value] * joint_count
        torque_target = tuple(
            float(value) for value in validate_command(model, values, "torques")
        )
    return angle_target, torque_target


def _wait_for_torque_dispatch(
    node: FrequencyBenchmarkNode,
    target: Sequence[float],
    timeout_sec: float,
) -> None:
    previous = node.latest_diagnostic
    previous_id = (
        None if previous is None else previous.values.get("latest_torques_command_id")
    )
    node.command_client.send_torques(target)

    def completed() -> bool:
        observation = node.latest_diagnostic
        if observation is None:
            return False
        command_id = observation.values.get("latest_torques_command_id")
        stage = observation.values.get("latest_torques_stage")
        return command_id not in (None, "none", previous_id) and stage in {
            "dispatched",
            "acknowledged",
            "failed",
            "superseded",
        }

    if not _spin_until(node, completed, timeout_sec):
        raise RuntimeError("等待力矩或电流限制实际 dispatch 超时")
    stage = node.latest_diagnostic.values.get("latest_torques_stage")
    if stage not in {"dispatched", "acknowledged"}:
        raise RuntimeError(f"力矩或电流限制未成功发送，终态为 {stage}")


def _angle_values(
    baseline: tuple[float, ...],
    elapsed: float,
    parsed: argparse.Namespace,
) -> tuple[float, ...]:
    if parsed.angle_amplitude == 0.0:
        return baseline
    values = list(baseline)
    offset = parsed.angle_amplitude * math.sin(
        2.0 * math.pi * elapsed / parsed.angle_period
    )
    values[parsed.joint_index] = min(
        100.0,
        max(0.0, baseline[parsed.joint_index] + offset),
    )
    return tuple(values)


def _command_for_tick(mode: str, tick: int) -> str:
    if mode in {"control", "angles"}:
        return "angles"
    if mode == "torques":
        return "torques"
    return "angles" if tick % 2 == 0 else "torques"


def _run_window(
    node: FrequencyBenchmarkNode,
    parsed: argparse.Namespace,
    rate_hz: float,
    required_sensors: tuple[str, ...],
    angle_target: tuple[float, ...] | None,
    torque_target: tuple[float, ...] | None,
) -> BenchmarkResult:
    start_diagnostic = node.latest_diagnostic
    start_generation = (
        None
        if start_diagnostic is None
        else start_diagnostic.values.get("connection_generation")
    )
    started_at = time.monotonic()
    ended_at = started_at + parsed.duration
    next_deadline = started_at
    sent_at: list[float] = []
    command_errors: list[str] = []
    skipped_deadlines = 0
    tick = 0
    period = 1.0 / rate_hz

    while rclpy.ok() and time.monotonic() < ended_at:
        latest_diagnostic = node.latest_diagnostic
        if latest_diagnostic is not None and (
            latest_diagnostic.values.get("connected", "").lower() != "true"
            or latest_diagnostic.values.get("connection_generation") != start_generation
        ):
            command_errors.append("连接中断或代次变化，已停止提交后续目标")
            break
        now = time.monotonic()
        timeout = min(0.005, max(0.0, next_deadline - now), ended_at - now)
        rclpy.spin_once(node, timeout_sec=timeout)
        now = time.monotonic()
        if now < next_deadline or now >= ended_at:
            continue

        command = _command_for_tick(parsed.mode, tick)
        try:
            if command == "angles":
                if angle_target is None:
                    raise RuntimeError("缺少角度目标")
                values = _angle_values(angle_target, now - started_at, parsed)
                node.command_client.send_angles(values)
            else:
                if torque_target is None:
                    raise RuntimeError("缺少力矩目标")
                node.command_client.send_torques(torque_target)
            sent_at.append(time.monotonic())
        except (RuntimeError, ValueError) as error:
            command_errors.append(str(error))

        tick += 1
        next_deadline += period
        if next_deadline <= now:
            missed = math.floor((now - next_deadline) / period) + 1
            skipped_deadlines += missed
            next_deadline += missed * period

    actual_end = min(time.monotonic(), ended_at)
    drain_deadline = time.monotonic() + parsed.drain_time
    while rclpy.ok() and time.monotonic() < drain_deadline:
        rclpy.spin_once(
            node,
            timeout_sec=min(0.05, drain_deadline - time.monotonic()),
        )

    accepted = tuple(
        event
        for event in node.accepted_events
        if started_at <= event[1] <= drain_deadline
    )
    diagnostics = tuple(
        observation
        for observation in node.diagnostic_observations
        if started_at <= observation.observed_at <= actual_end
    )
    command_names = (
        ("angles", "torques")
        if parsed.mode == "mixed"
        else (_command_for_tick(parsed.mode, 0),)
    )
    return evaluate_measurements(
        BenchmarkMeasurements(
            requested_hz=rate_hz,
            started_at=started_at,
            ended_at=actual_end,
            command_names=command_names,
            required_sensors=required_sensors,
            sent_at=tuple(sent_at),
            accepted_commands=accepted,
            state_events=tuple(node.state_events),
            diagnostic_samples=diagnostics,
            start_diagnostic=start_diagnostic,
            end_diagnostic=node.latest_diagnostic,
            command_errors=tuple(command_errors),
            skipped_deadlines=skipped_deadlines,
        )
    )


def _log_result(
    node: Node,
    result: BenchmarkResult,
    repetition: int,
) -> None:
    status = "PASS" if result.passed else "FAIL"
    state_text = ", ".join(
        f"{sensor}_topic={rate:.2f}Hz" for sensor, rate in result.state_hz
    )
    sample_text = ", ".join(
        f"{sensor}_sample={'N/A' if rate is None else f'{rate:.2f}Hz'}"
        for sensor, rate in result.sample_hz
    )
    p95 = (
        "N/A"
        if result.send_interval_p95_ms is None
        else f"{result.send_interval_p95_ms:.2f}ms"
    )
    superseded = (
        "N/A" if result.superseded_ratio is None else f"{result.superseded_ratio:.2%}"
    )
    node.get_logger().info(
        f"[{status}] {result.requested_hz:g}Hz 第{repetition}轮: "
        f"send={result.send_hz:.2f}Hz, p95={p95}, "
        f"accepted={result.accepted_count}/{result.sent_count} "
        f"({result.accepted_ratio:.2%}), superseded={superseded}, "
        f"pending={result.pending_commands}, {state_text}, {sample_text}"
    )
    for reason in result.reasons:
        node.get_logger().error(f"  - {reason}")


def _run(parsed: argparse.Namespace, node: FrequencyBenchmarkNode) -> bool:
    if not spin_until_ready(node, node.command_client, parsed.startup_timeout):
        raise RuntimeError(f"等待驱动角度接口就绪超时: {node.driver_name}")
    model = node.command_client.model
    if model is None:
        raise RuntimeError("未收到驱动型号")
    secondary_sensor = _secondary_sensor(model)
    required_sensors = (
        ("angles", secondary_sensor)
        if parsed.feedback == "angles-and-load"
        else ("angles",)
    )

    if not _spin_until(
        node,
        lambda: node.latest_diagnostic is not None and "angles" in node.seen_sensors,
        parsed.startup_timeout,
    ):
        raise RuntimeError("等待首次完整角度状态和诊断超时")
    angle_target, torque_target = _resolve_targets(parsed, node)
    if parsed.mode == "control":
        _wait_for_torque_dispatch(node, torque_target, parsed.startup_timeout)

    node.get_logger().info(
        f"型号={model}, 关节数={len(node.command_client.joint_names)}, "
        f"第二反馈={secondary_sensor}; mixed 模式的频率是两类命令总频率"
    )

    highest_stable: float | None = None
    all_passed = True
    for rate_hz in parsed.rates:
        if not parsed.no_configure_driver:
            configured_secondary = (
                secondary_sensor if parsed.feedback == "angles-and-load" else None
            )
            node.configure_rates(
                rate_hz,
                configured_secondary,
                parsed.startup_timeout,
            )
        warmup_deadline = time.monotonic() + parsed.warmup
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(
                node,
                timeout_sec=min(0.05, warmup_deadline - time.monotonic()),
            )

        results = []
        for repetition in range(1, parsed.repetitions + 1):
            result = _run_window(
                node,
                parsed,
                rate_hz,
                required_sensors,
                angle_target,
                torque_target,
            )
            results.append(result)
            _log_result(node, result, repetition)
            if any("连接" in reason for reason in result.reasons):
                raise RuntimeError("连接在测试期间变化，已中止剩余频率测试")
        stable = all(result.passed for result in results)
        if stable:
            highest_stable = rate_hz
        else:
            all_passed = False
            if not parsed.continue_after_failure:
                break

    if highest_stable is None:
        node.get_logger().error("没有候选频率连续通过全部轮次")
    else:
        node.get_logger().info(f"稳定频率上限（候选集内）: {highest_stable:g} Hz")
    return all_passed and highest_stable == parsed.rates[-1]


def main(args: list[str] | None = None) -> None:
    """Run the command and telemetry frequency benchmark."""
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    try:
        parsed = _parse_args(remove_ros_args(raw_args)[1:])
        _validate_args(parsed)
        driver_name = resolve_driver_name(parsed.driver_name)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    rclpy.init(args=raw_args)
    node = FrequencyBenchmarkNode(driver_name)
    exit_code = 1
    try:
        exit_code = 0 if _run(parsed, node) else 2
    except KeyboardInterrupt:
        exit_code = 130
    except (RuntimeError, ValueError) as error:
        node.get_logger().error(str(error))
    finally:
        node.destroy_benchmark()
        node.destroy_node()
        rclpy.try_shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
