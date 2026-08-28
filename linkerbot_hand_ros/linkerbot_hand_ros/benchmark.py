"""Pure evaluation for equal-workload ROS angle and tactile benchmarks."""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

from linkerbot_hand_msgs.msg import HandCommandStatus


@dataclass(frozen=True, slots=True)
class StateEvent:
    channel: str
    observed_at: float
    sequence: int
    generation: int
    valid: bool
    stale: bool


@dataclass(frozen=True, slots=True)
class CommandEvent:
    observed_at: float
    user_sequence: int
    stage: int
    ack_support: int


@dataclass(frozen=True, slots=True)
class DiagnosticObservation:
    observed_at: float
    values: dict[str, str]

    def integer(self, key: str) -> int | None:
        value = self.values.get(key)
        if value is None:
            return None
        try:
            return int(value)
        except ValueError:
            return None


@dataclass(frozen=True, slots=True)
class BenchmarkMeasurements:
    requested_angle_hz: float
    requested_tactile_hz: float
    started_at: float
    ended_at: float
    sent_at: tuple[float, ...]
    state_events: tuple[StateEvent, ...]
    command_events: tuple[CommandEvent, ...]
    start_diagnostic: DiagnosticObservation | None
    end_diagnostic: DiagnosticObservation | None


@dataclass(frozen=True, slots=True)
class BenchmarkResult:
    requested_angle_hz: float
    requested_tactile_hz: float
    passed: bool
    command_publish_hz: float
    command_terminal_hz: float
    command_dispatch_hz: float
    angle_sample_hz: float | None
    tactile_sample_hz: float | None
    command_interval_p95_ms: float | None
    failed_commands: int
    superseded_commands: int
    reasons: tuple[str, ...]


def evaluate_measurements(
    measurements: BenchmarkMeasurements,
    *,
    minimum_ratio: float = 0.9,
) -> BenchmarkResult:
    """Evaluate one candidate using device sequences and protocol stages."""

    duration = measurements.ended_at - measurements.started_at
    if duration <= 0:
        raise ValueError("benchmark duration must be positive")
    if not 0 < minimum_ratio <= 1:
        raise ValueError("minimum_ratio must be within (0, 1]")
    publish_hz = len(measurements.sent_at) / duration
    terminal = tuple(
        event for event in measurements.command_events if _is_protocol_terminal(event)
    )
    terminal_hz = len({event.user_sequence for event in terminal}) / duration
    dispatched_hz = (
        len(
            {
                event.user_sequence
                for event in measurements.command_events
                if _is_dispatched(event)
            }
        )
        / duration
    )
    failed = len(
        {
            event.user_sequence
            for event in terminal
            if event.stage == HandCommandStatus.STAGE_FAILED
        }
    )
    superseded = len(
        {
            event.user_sequence
            for event in terminal
            if event.stage == HandCommandStatus.STAGE_SUPERSEDED
        }
    )
    angle_hz, angle_generation_changed = _sample_rate(
        measurements.state_events,
        "angles",
    )
    tactile_hz, tactile_generation_changed = _sample_rate(
        measurements.state_events,
        "tactile",
    )
    reasons: list[str] = []
    minimum_angle = measurements.requested_angle_hz * minimum_ratio
    if publish_hz < minimum_angle:
        reasons.append(f"命令发布频率 {publish_hz:.2f} Hz 低于 {minimum_angle:.2f} Hz")
    if terminal_hz < minimum_angle:
        reasons.append(
            f"命令协议终态频率 {terminal_hz:.2f} Hz 低于 {minimum_angle:.2f} Hz"
        )
    if dispatched_hz < minimum_angle:
        reasons.append(
            f"命令实际下发频率 {dispatched_hz:.2f} Hz 低于 {minimum_angle:.2f} Hz"
        )
    if angle_hz is None or angle_hz < minimum_angle:
        value = "unknown" if angle_hz is None else f"{angle_hz:.2f} Hz"
        reasons.append(f"角度设备采样频率 {value} 低于 {minimum_angle:.2f} Hz")
    if measurements.requested_tactile_hz > 0:
        minimum_tactile = measurements.requested_tactile_hz * minimum_ratio
        if tactile_hz is None or tactile_hz < minimum_tactile:
            value = "unknown" if tactile_hz is None else f"{tactile_hz:.2f} Hz"
            reasons.append(f"完整触觉采样频率 {value} 低于 {minimum_tactile:.2f} Hz")
    if angle_generation_changed or tactile_generation_changed:
        reasons.append("测试期间连接代次发生变化")
    if failed:
        reasons.append(f"存在 {failed} 个 FAILED 命令")
    _append_health_failures(measurements, reasons)
    return BenchmarkResult(
        requested_angle_hz=measurements.requested_angle_hz,
        requested_tactile_hz=measurements.requested_tactile_hz,
        passed=not reasons,
        command_publish_hz=publish_hz,
        command_terminal_hz=terminal_hz,
        command_dispatch_hz=dispatched_hz,
        angle_sample_hz=angle_hz,
        tactile_sample_hz=tactile_hz,
        command_interval_p95_ms=_interval_p95_ms(measurements.sent_at),
        failed_commands=failed,
        superseded_commands=superseded,
        reasons=tuple(reasons),
    )


def _is_protocol_terminal(event: CommandEvent) -> bool:
    return event.stage in {
        HandCommandStatus.STAGE_ACKNOWLEDGED,
        HandCommandStatus.STAGE_FAILED,
        HandCommandStatus.STAGE_SUPERSEDED,
    } or (
        event.stage == HandCommandStatus.STAGE_DISPATCHED
        and event.ack_support == HandCommandStatus.ACK_UNSUPPORTED
    )


def _is_dispatched(event: CommandEvent) -> bool:
    return event.stage in {
        HandCommandStatus.STAGE_DISPATCHED,
        HandCommandStatus.STAGE_ACKNOWLEDGED,
    }


def _sample_rate(
    events: tuple[StateEvent, ...],
    channel: str,
) -> tuple[float | None, bool]:
    channel_events = tuple(event for event in events if event.channel == channel)
    generations = {event.generation for event in channel_events}
    if len(generations) > 1:
        return None, True
    valid = tuple(event for event in channel_events if event.valid and not event.stale)
    if len(valid) < 2:
        return None, False
    elapsed = valid[-1].observed_at - valid[0].observed_at
    sequence_delta = valid[-1].sequence - valid[0].sequence
    if elapsed <= 0 or sequence_delta < 0:
        return None, False
    return sequence_delta / elapsed, False


def _append_health_failures(
    measurements: BenchmarkMeasurements,
    reasons: list[str],
) -> None:
    start = measurements.start_diagnostic
    end = measurements.end_diagnostic
    if start is None or end is None:
        reasons.append("缺少测试起止 health diagnostics")
        return
    if start.values.get("connection_generation") != end.values.get(
        "connection_generation"
    ):
        reasons.append("测试期间 health 连接代次发生变化")
    pending = end.integer("pending_commands")
    if pending is None:
        reasons.append("诊断中缺少 pending_commands")
    elif pending:
        reasons.append(f"测试结束仍有 {pending} 个待发送命令")
    consecutive_errors = end.integer("consecutive_errors")
    if consecutive_errors is None:
        reasons.append("诊断中缺少 consecutive_errors")
    elif consecutive_errors:
        reasons.append(f"连续 SDK 错误当前为 {consecutive_errors}")
    for key, label in (
        ("transport_dropped_frames", "transport 丢帧"),
        ("transport_route_misses", "transport 路由丢失"),
        ("transport_bus_errors", "CAN 总线错误"),
        ("channel_angles_timeouts", "角度事务超时"),
    ):
        delta = _counter_delta(start, end, key)
        if delta is None:
            reasons.append(f"诊断中缺少或重置 {key}")
        elif delta:
            reasons.append(f"{label}增加 {delta}")
    if measurements.requested_tactile_hz > 0:
        delta = _counter_delta(start, end, "channel_force_timeouts")
        if delta is None:
            reasons.append("诊断中缺少或重置 channel_force_timeouts")
        elif delta:
            reasons.append(f"触觉事务超时增加 {delta}")


def _counter_delta(
    start: DiagnosticObservation,
    end: DiagnosticObservation,
    key: str,
) -> int | None:
    start_value = start.integer(key)
    end_value = end.integer(key)
    if start_value is None or end_value is None or end_value < start_value:
        return None
    return end_value - start_value


def _interval_p95_ms(timestamps: tuple[float, ...]) -> float | None:
    intervals = sorted(later - earlier for earlier, later in pairwise(timestamps))
    if not intervals:
        return None
    index = max(0, math.ceil(len(intervals) * 0.95) - 1)
    return intervals[index] * 1000


__all__ = [
    "BenchmarkMeasurements",
    "BenchmarkResult",
    "CommandEvent",
    "DiagnosticObservation",
    "StateEvent",
    "evaluate_measurements",
]
