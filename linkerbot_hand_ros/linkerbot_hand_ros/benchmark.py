"""Pure measurements and pass criteria for hand frequency benchmarks."""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

SUCCESSFUL_COMMAND_STAGES = frozenset({"dispatched", "acknowledged"})


@dataclass(frozen=True)
class DiagnosticObservation:
    """One parsed driver status and its local receipt time."""

    observed_at: float
    values: dict[str, str]

    def integer(self, key: str) -> int | None:
        """Return an integer diagnostic value or ``None`` when unavailable."""
        try:
            return int(self.values[key])
        except (KeyError, TypeError, ValueError):
            return None


@dataclass(frozen=True)
class StateEvent:
    """Receipt time and structural validity of one ROS state message."""

    sensor: str
    observed_at: float
    valid: bool


@dataclass(frozen=True)
class BenchmarkMeasurements:
    """Immutable inputs used to evaluate one frequency window."""

    requested_hz: float
    started_at: float
    ended_at: float
    command_names: tuple[str, ...]
    required_sensors: tuple[str, ...]
    sent_at: tuple[float, ...]
    accepted_commands: tuple[tuple[str, float], ...]
    state_events: tuple[StateEvent, ...]
    diagnostic_samples: tuple[DiagnosticObservation, ...]
    start_diagnostic: DiagnosticObservation | None
    end_diagnostic: DiagnosticObservation | None
    command_errors: tuple[str, ...] = ()
    skipped_deadlines: int = 0


@dataclass(frozen=True)
class BenchmarkResult:
    """Calculated rates and deterministic acceptance decision for one window."""

    passed: bool
    reasons: tuple[str, ...]
    requested_hz: float
    send_hz: float
    send_interval_p95_ms: float | None
    sent_count: int
    accepted_count: int
    accepted_ratio: float
    superseded_count: int | None
    superseded_ratio: float | None
    pending_commands: int | None
    state_hz: tuple[tuple[str, float], ...]
    sample_hz: tuple[tuple[str, float | None], ...]
    skipped_deadlines: int


def evaluate_measurements(
    measurements: BenchmarkMeasurements,
    *,
    minimum_delivery_ratio: float = 0.95,
    minimum_feedback_ratio: float = 0.90,
    maximum_superseded_ratio: float = 0.01,
) -> BenchmarkResult:
    """Apply the documented stability criteria to one test window."""
    elapsed = measurements.ended_at - measurements.started_at
    if elapsed <= 0.0:
        raise ValueError("benchmark window must have positive duration")

    sent_count = len(measurements.sent_at)
    accepted_count = sum(
        command in measurements.command_names
        for command, _observed_at in measurements.accepted_commands
    )
    send_hz = sent_count / elapsed
    accepted_ratio = accepted_count / sent_count if sent_count else 0.0
    state_rates = tuple(
        (
            sensor,
            sum(
                event.sensor == sensor
                and measurements.started_at
                <= event.observed_at
                <= measurements.ended_at
                for event in measurements.state_events
            )
            / elapsed,
        )
        for sensor in measurements.required_sensors
    )
    sample_rates = tuple(
        (
            sensor,
            _sensor_sample_rate(measurements.diagnostic_samples, sensor),
        )
        for sensor in measurements.required_sensors
    )

    start = measurements.start_diagnostic
    end = measurements.end_diagnostic
    superseded = _counter_delta(start, end, "superseded_commands")
    superseded_ratio = (
        superseded / sent_count if superseded is not None and sent_count else None
    )
    pending = None if end is None else end.integer("pending_commands")
    reasons: list[str] = []

    if send_hz < measurements.requested_hz * minimum_delivery_ratio:
        reasons.append(
            f"发送频率 {send_hz:.2f} Hz 低于目标的 {minimum_delivery_ratio:.0%}"
        )
    if accepted_ratio < minimum_delivery_ratio:
        reasons.append(
            f"ROS accepted 比例 {accepted_ratio:.2%} 低于 {minimum_delivery_ratio:.0%}"
        )
    if accepted_count > sent_count:
        reasons.append("accepted 数量大于本程序发送数，请停止其他命令发布者")
    if measurements.command_errors:
        reasons.append(f"发生 {len(measurements.command_errors)} 次命令发送异常")

    invalid_states = sum(
        not event.valid
        and event.sensor in measurements.required_sensors
        and measurements.started_at <= event.observed_at <= measurements.ended_at
        for event in measurements.state_events
    )
    if invalid_states:
        reasons.append(f"收到 {invalid_states} 条长度错误或非有限状态")

    for sensor, state_rate in state_rates:
        if state_rate < measurements.requested_hz * minimum_feedback_ratio:
            reasons.append(
                f"{sensor} ROS 状态频率 {state_rate:.2f} Hz 低于目标的 "
                f"{minimum_feedback_ratio:.0%}"
            )
    for sensor, sample_rate in sample_rates:
        if sample_rate is None:
            reasons.append(f"{sensor} 缺少同一连接代次内的采样序号")
        elif sample_rate < measurements.requested_hz * minimum_feedback_ratio:
            reasons.append(
                f"{sensor} 设备采样频率 {sample_rate:.2f} Hz 低于目标的 "
                f"{minimum_feedback_ratio:.0%}"
            )

    if start is None or end is None:
        reasons.append("测试窗口缺少驱动诊断")
    else:
        _evaluate_health(
            start,
            end,
            measurements.command_names,
            superseded_ratio,
            maximum_superseded_ratio,
            reasons,
        )
    if pending is None:
        reasons.append("诊断中缺少待发送命令数")
    elif pending != 0:
        reasons.append(f"等待排空后仍有 {pending} 条待发送命令")

    return BenchmarkResult(
        passed=not reasons,
        reasons=tuple(reasons),
        requested_hz=measurements.requested_hz,
        send_hz=send_hz,
        send_interval_p95_ms=_interval_p95_ms(measurements.sent_at),
        sent_count=sent_count,
        accepted_count=accepted_count,
        accepted_ratio=accepted_ratio,
        superseded_count=superseded,
        superseded_ratio=superseded_ratio,
        pending_commands=pending,
        state_hz=state_rates,
        sample_hz=sample_rates,
        skipped_deadlines=measurements.skipped_deadlines,
    )


def _evaluate_health(
    start: DiagnosticObservation,
    end: DiagnosticObservation,
    command_names: tuple[str, ...],
    superseded_ratio: float | None,
    maximum_superseded_ratio: float,
    reasons: list[str],
) -> None:
    start_generation = start.values.get("connection_generation")
    end_generation = end.values.get("connection_generation")
    if start_generation is None or end_generation is None:
        reasons.append("诊断中缺少连接代次")
    elif start_generation != end_generation:
        reasons.append("测试期间连接代次发生变化")
    if end.values.get("connected", "").lower() != "true":
        reasons.append("测试结束时设备未连接")
    consecutive_errors = end.integer("consecutive_errors")
    if consecutive_errors is None:
        reasons.append("诊断中缺少连续错误计数")
    elif consecutive_errors != 0:
        reasons.append("测试结束时存在连续通信错误")

    for counter in ("ros_commands_rejected", "ros_accepted_feedback_errors"):
        delta = _counter_delta(start, end, counter)
        if delta is None:
            reasons.append(f"诊断中缺少 {counter}")
        elif delta != 0:
            reasons.append(f"{counter} 增加 {delta}")
    if superseded_ratio is None:
        reasons.append("诊断中缺少命令覆盖计数")
    elif superseded_ratio > maximum_superseded_ratio:
        reasons.append(
            f"命令覆盖比例 {superseded_ratio:.2%} 超过 {maximum_superseded_ratio:.0%}"
        )

    for command in command_names:
        stage = end.values.get(f"latest_{command}_stage")
        if stage not in SUCCESSFUL_COMMAND_STAGES:
            reasons.append(f"{command} 最新命令终态为 {stage or 'unknown'}")


def _counter_delta(
    start: DiagnosticObservation | None,
    end: DiagnosticObservation | None,
    key: str,
) -> int | None:
    if start is None or end is None:
        return None
    start_value = start.integer(key)
    end_value = end.integer(key)
    if start_value is None or end_value is None or end_value < start_value:
        return None
    return end_value - start_value


def _sensor_sample_rate(
    observations: tuple[DiagnosticObservation, ...], sensor: str
) -> float | None:
    points: list[tuple[float, int, str]] = []
    for observation in observations:
        sequence = observation.integer(f"{sensor}_sequence")
        generation = observation.values.get(f"{sensor}_connection_generation")
        if sequence is not None and generation is not None:
            points.append((observation.observed_at, sequence, generation))
    if len(points) < 2:
        return None
    first = points[0]
    last = points[-1]
    elapsed = last[0] - first[0]
    sequence_delta = last[1] - first[1]
    if first[2] != last[2] or elapsed <= 0.0 or sequence_delta < 0:
        return None
    return sequence_delta / elapsed


def _interval_p95_ms(timestamps: tuple[float, ...]) -> float | None:
    intervals = sorted(later - earlier for earlier, later in pairwise(timestamps))
    if not intervals:
        return None
    index = max(0, math.ceil(len(intervals) * 0.95) - 1)
    return intervals[index] * 1000.0


__all__ = [
    "BenchmarkMeasurements",
    "BenchmarkResult",
    "DiagnosticObservation",
    "StateEvent",
    "evaluate_measurements",
]
