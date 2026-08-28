"""Measure equal-workload ROS angle and optional complete tactile rates."""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable, Sequence
from itertools import pairwise

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.utilities import remove_ros_args

from linkerbot_hand_msgs.msg import (
    HandCommandStatus,
    HandJointState,
    HandTactileState,
)
from linkerbot_hand_ros.benchmark import (
    BenchmarkMeasurements,
    BenchmarkResult,
    CommandEvent,
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
from linkerbot_hand_ros.qos import STATE_QOS, STATUS_QOS

DEFAULT_ANGLE_RATES = (50.0, 100.0, 150.0, 200.0)


def _parse_args(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="测量 ROS 百分比角度收发及可选完整触觉事务的稳定频率"
    )
    parser.add_argument("--driver-name", default=DEFAULT_DRIVER_NAME)
    parser.add_argument(
        "--angle-rates",
        type=float,
        nargs="+",
        default=list(DEFAULT_ANGLE_RATES),
        help="按升序测试的角度命令及角度轮询候选频率",
    )
    parser.add_argument(
        "--tactile-hz",
        type=float,
        default=0.0,
        help="完整触觉轮询频率；0 表示仅测试角度",
    )
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--drain-time", type=float, default=1.0)
    parser.add_argument("--startup-timeout", type=float, default=10.0)
    parser.add_argument("--minimum-ratio", type=float, default=0.9)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument(
        "--continue-after-failure",
        action="store_true",
        help="某一候选频率失败后仍继续测试更高频率",
    )
    parser.add_argument(
        "--no-configure-driver",
        action="store_true",
        help="不自动同步 SDK polling、ROS 发布和诊断频率",
    )
    parser.add_argument(
        "--angle-values",
        type=float,
        nargs="+",
        help="百分比角度基准；省略时保持首次收到的有效 percent 状态",
    )
    parser.add_argument(
        "--angle-amplitude",
        type=float,
        default=0.0,
        help="单关节百分比正弦扰动幅度；0 表示保持当前姿态",
    )
    parser.add_argument("--angle-period", type=float, default=2.0)
    parser.add_argument("--joint-index", type=int, default=0)
    return parser.parse_args(args)


def _validate_args(parsed: argparse.Namespace) -> None:
    rates = parsed.angle_rates
    if not rates or any(not math.isfinite(rate) or rate <= 0.0 for rate in rates):
        raise ValueError("--angle-rates 必须是有限正数")
    if any(later <= earlier for earlier, later in pairwise(rates)):
        raise ValueError("--angle-rates 必须严格递增且不能重复")
    for name in ("duration", "drain_time", "startup_timeout"):
        value = getattr(parsed, name)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"--{name.replace('_', '-')} 必须是有限正数")
    if not math.isfinite(parsed.warmup) or parsed.warmup < 0.0:
        raise ValueError("--warmup 必须是有限非负数")
    if not 0.0 < parsed.minimum_ratio <= 1.0:
        raise ValueError("--minimum-ratio 必须在 (0, 1] 内")
    if not math.isfinite(parsed.tactile_hz) or parsed.tactile_hz < 0.0:
        raise ValueError("--tactile-hz 必须是有限非负数")
    if type(parsed.repetitions) is not int or parsed.repetitions <= 0:
        raise ValueError("--repetitions 必须大于 0")
    if type(parsed.joint_index) is not int or parsed.joint_index < 0:
        raise ValueError("--joint-index 不能为负数")
    if not math.isfinite(parsed.angle_amplitude) or parsed.angle_amplitude < 0.0:
        raise ValueError("--angle-amplitude 必须是有限非负数")
    if not math.isfinite(parsed.angle_period) or parsed.angle_period <= 0.0:
        raise ValueError("--angle-period 必须是有限正数")


class FrequencyBenchmarkNode(Node):
    """Own persistent typed clients and observations for the benchmark."""

    def __init__(self, driver_name: str) -> None:
        super().__init__("linkerbot_hand_frequency_benchmark")
        self.driver_name = driver_name.rstrip("/")
        self.command_client = HandCommandClient(self, self.driver_name)
        self.parameter_client = AsyncParameterClient(self, self.driver_name)
        self.latest_percent_angles: tuple[float, ...] | None = None
        self.state_events: list[StateEvent] = []
        self.command_events: list[CommandEvent] = []
        self.diagnostic_observations: list[DiagnosticObservation] = []
        self._observation_subscriptions = []

    def start_observing(self) -> None:
        """Subscribe to high-rate data after capability discovery completes."""

        if self._observation_subscriptions:
            return
        subscriptions = []
        try:
            specifications = (
                (
                    HandJointState,
                    f"{self.driver_name}/state/angles",
                    self._on_angles,
                    STATE_QOS,
                ),
                (
                    HandTactileState,
                    f"{self.driver_name}/state/tactile",
                    self._on_tactile,
                    STATE_QOS,
                ),
                (
                    HandCommandStatus,
                    f"{self.driver_name}/command/status",
                    self._on_command_status,
                    STATUS_QOS,
                ),
                (
                    DiagnosticArray,
                    "/diagnostics",
                    self._on_diagnostics,
                    STATE_QOS,
                ),
            )
            for message_type, topic, callback, qos in specifications:
                subscriptions.append(
                    self.create_subscription(message_type, topic, callback, qos)
                )
        except Exception:
            for subscription in subscriptions:
                self.destroy_subscription(subscription)
            raise
        self._observation_subscriptions = subscriptions

    @property
    def latest_diagnostic(self) -> DiagnosticObservation | None:
        if not self.diagnostic_observations:
            return None
        return self.diagnostic_observations[-1]

    def _on_angles(self, message: HandJointState) -> None:
        observed_at = time.monotonic()
        joint_count = len(message.joint_names)
        structurally_valid = (
            joint_count > 0
            and len(message.raw) == joint_count
            and len(message.percent) == joint_count
            and len(message.percent_valid) == joint_count
            and len(message.physical_rad) == joint_count
            and len(message.physical_rad_valid) == joint_count
        )
        if (
            structurally_valid
            and message.meta.valid
            and not message.meta.stale
            and all(message.percent_valid)
            and all(math.isfinite(value) for value in message.percent)
        ):
            self.latest_percent_angles = tuple(message.percent)
        self.state_events.append(
            StateEvent(
                channel="angles",
                observed_at=observed_at,
                sequence=message.meta.sequence,
                generation=message.meta.connection_generation,
                valid=message.meta.valid and structurally_valid,
                stale=message.meta.stale,
            )
        )

    def _on_tactile(self, message: HandTactileState) -> None:
        structurally_valid = bool(message.fingers) and all(
            matrix.rows > 0
            and matrix.columns > 0
            and len(matrix.raw) == matrix.rows * matrix.columns
            for matrix in message.fingers
        )
        self.state_events.append(
            StateEvent(
                channel="tactile",
                observed_at=time.monotonic(),
                sequence=message.meta.sequence,
                generation=message.meta.connection_generation,
                valid=message.meta.valid and structurally_valid,
                stale=message.meta.stale,
            )
        )

    def _on_command_status(self, message: HandCommandStatus) -> None:
        if message.user_sequence == 0:
            return
        self.command_events.append(
            CommandEvent(
                observed_at=time.monotonic(),
                user_sequence=message.user_sequence,
                stage=message.stage,
                ack_support=message.ack_support,
            )
        )

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
        angle_hz: float,
        tactile_hz: float,
        timeout_s: float,
    ) -> None:
        """Atomically align SDK polling and ROS cache publication rates."""

        if not self.parameter_client.wait_for_services(timeout_sec=timeout_s):
            raise RuntimeError(f"参数服务未就绪: {self.driver_name}")
        publish_hz = max(angle_hz, tactile_hz)
        parameters = [
            Parameter("angles_hz", value=angle_hz),
            Parameter("force_hz", value=tactile_hz),
            Parameter("publish_rate_hz", value=publish_hz),
            Parameter("diagnostic_rate_hz", value=min(publish_hz, 20.0)),
        ]
        future = self.parameter_client.set_parameters_atomically(parameters)
        _spin_until_future(self, future, timeout_s)
        response = future.result()
        if response is None or not response.result.successful:
            reason = "无响应" if response is None else response.result.reason
            raise RuntimeError(f"驱动频率配置失败: {reason}")

    def destroy_benchmark(self) -> None:
        self.command_client.destroy()
        for subscription in self._observation_subscriptions:
            self.destroy_subscription(subscription)
        self._observation_subscriptions.clear()


def _spin_until_future(node: Node, future: object, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError("等待 ROS 服务响应超时")


def _spin_until(
    node: Node,
    predicate: Callable[[], bool],
    timeout_s: float,
) -> bool:
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=min(0.05, deadline - time.monotonic()))
    return predicate()


def _resolve_angle_target(
    parsed: argparse.Namespace,
    node: FrequencyBenchmarkNode,
) -> tuple[float, ...]:
    values = parsed.angle_values or node.latest_percent_angles
    if values is None:
        raise RuntimeError(
            "没有完整有效的 percent 角度；请检查轮询或显式传 --angle-values"
        )
    target = tuple(float(value) for value in values)
    joint_count = len(node.command_client.joint_names)
    if len(target) != joint_count:
        raise ValueError(f"--angle-values 必须提供 {joint_count} 个值")
    if parsed.joint_index >= joint_count:
        raise ValueError(f"--joint-index 必须小于关节数量 {joint_count}")
    if any(not math.isfinite(value) or not 0.0 <= value <= 100.0 for value in target):
        raise ValueError("--angle-values 必须是 0..100 内的有限值")
    return target


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


def _warm_up(
    node: FrequencyBenchmarkNode,
    duration_s: float,
    *,
    require_tactile: bool,
    state_start: int,
) -> None:
    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, deadline - time.monotonic()))
    if not require_tactile:
        return
    new_events = node.state_events[state_start:]
    if not any(
        event.channel == "tactile" and event.valid and not event.stale
        for event in new_events
    ):
        raise RuntimeError("预热期间未收到完整有效的触觉事务")


def _run_window(
    node: FrequencyBenchmarkNode,
    parsed: argparse.Namespace,
    angle_hz: float,
    baseline: tuple[float, ...],
) -> BenchmarkResult:
    start_diagnostic = node.latest_diagnostic
    if start_diagnostic is None:
        raise RuntimeError("测试开始时缺少 health diagnostics")
    start_generation = start_diagnostic.values.get("connection_generation")
    state_start = len(node.state_events)
    command_start = len(node.command_events)
    started_at = time.monotonic()
    deadline = started_at + parsed.duration
    next_command_at = started_at
    period = 1.0 / angle_hz
    sent_at: list[float] = []
    sent_sequences: set[int] = set()

    while rclpy.ok() and time.monotonic() < deadline:
        diagnostic = node.latest_diagnostic
        if diagnostic is not None and (
            diagnostic.values.get("connected", "").lower() != "true"
            or diagnostic.values.get("connection_generation") != start_generation
        ):
            break
        now = time.monotonic()
        timeout_s = min(0.005, max(0.0, next_command_at - now), deadline - now)
        rclpy.spin_once(node, timeout_sec=timeout_s)
        now = time.monotonic()
        if now < next_command_at or now >= deadline:
            continue
        values = _angle_values(baseline, now - started_at, parsed)
        publication = node.command_client.set_percent_angles(values)
        sent_at.append(time.monotonic())
        sent_sequences.add(publication.user_sequence)
        next_command_at += period
        if next_command_at <= now:
            missed = math.floor((now - next_command_at) / period) + 1
            next_command_at += missed * period

    ended_at = min(time.monotonic(), deadline)
    drain_deadline = time.monotonic() + parsed.drain_time
    while rclpy.ok() and time.monotonic() < drain_deadline:
        rclpy.spin_once(
            node,
            timeout_sec=min(0.05, drain_deadline - time.monotonic()),
        )

    state_events = tuple(
        event
        for event in node.state_events[state_start:]
        if started_at <= event.observed_at <= ended_at
    )
    command_events = tuple(
        event
        for event in node.command_events[command_start:]
        if event.user_sequence in sent_sequences
        and started_at <= event.observed_at <= drain_deadline
    )
    return evaluate_measurements(
        BenchmarkMeasurements(
            requested_angle_hz=angle_hz,
            requested_tactile_hz=parsed.tactile_hz,
            started_at=started_at,
            ended_at=ended_at,
            sent_at=tuple(sent_at),
            state_events=state_events,
            command_events=command_events,
            start_diagnostic=start_diagnostic,
            end_diagnostic=node.latest_diagnostic,
        ),
        minimum_ratio=parsed.minimum_ratio,
    )


def _log_result(
    node: Node,
    result: BenchmarkResult,
    repetition: int,
) -> None:
    status = "PASS" if result.passed else "FAIL"
    angle_sample = (
        "N/A" if result.angle_sample_hz is None else f"{result.angle_sample_hz:.2f}Hz"
    )
    tactile_sample = (
        "N/A"
        if result.tactile_sample_hz is None
        else f"{result.tactile_sample_hz:.2f}Hz"
    )
    p95 = (
        "N/A"
        if result.command_interval_p95_ms is None
        else f"{result.command_interval_p95_ms:.2f}ms"
    )
    node.get_logger().info(
        f"[{status}] angle={result.requested_angle_hz:g}Hz "
        f"tactile={result.requested_tactile_hz:g}Hz 第{repetition}轮: "
        f"publish={result.command_publish_hz:.2f}Hz, "
        f"terminal={result.command_terminal_hz:.2f}Hz, "
        f"angle_sample={angle_sample}, tactile_sample={tactile_sample}, "
        f"command_p95={p95}, failed={result.failed_commands}, "
        f"superseded={result.superseded_commands}"
    )
    for reason in result.reasons:
        node.get_logger().error(f"  - {reason}")


def _run(parsed: argparse.Namespace, node: FrequencyBenchmarkNode) -> bool:
    if not spin_until_ready(node, node.command_client, parsed.startup_timeout):
        raise RuntimeError(f"等待驱动角度接口就绪超时: {node.driver_name}")
    node.start_observing()
    if not _spin_until(
        node,
        lambda: (
            node.latest_diagnostic is not None
            and node.latest_percent_angles is not None
        ),
        parsed.startup_timeout,
    ):
        raise RuntimeError("等待首次有效 percent 角度状态和 health diagnostics 超时")
    baseline = _resolve_angle_target(parsed, node)
    model = node.command_client.model
    node.get_logger().info(
        f"型号={model}, 关节数={len(node.command_client.joint_names)}, "
        f"完整触觉={parsed.tactile_hz:g}Hz"
    )

    highest_stable: float | None = None
    all_passed = True
    for angle_hz in parsed.angle_rates:
        if not parsed.no_configure_driver:
            node.configure_rates(
                angle_hz,
                parsed.tactile_hz,
                parsed.startup_timeout,
            )
        state_start = len(node.state_events)
        _warm_up(
            node,
            parsed.warmup,
            require_tactile=parsed.tactile_hz > 0,
            state_start=state_start,
        )

        results = []
        for repetition in range(1, parsed.repetitions + 1):
            result = _run_window(node, parsed, angle_hz, baseline)
            results.append(result)
            _log_result(node, result, repetition)
            if any("连接" in reason for reason in result.reasons):
                raise RuntimeError("连接在测试期间变化，已中止剩余频率测试")
        if all(result.passed for result in results):
            highest_stable = angle_hz
        else:
            all_passed = False
            if not parsed.continue_after_failure:
                break

    if highest_stable is None:
        node.get_logger().error("没有候选频率连续通过全部轮次")
    else:
        node.get_logger().info(f"稳定角度频率上限（候选集内）: {highest_stable:g} Hz")
    return all_passed and highest_stable == parsed.angle_rates[-1]


def main(args: Sequence[str] | None = None) -> None:
    """Run the typed angle and complete tactile frequency benchmark."""

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
