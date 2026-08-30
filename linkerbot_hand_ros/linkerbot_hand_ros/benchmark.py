"""Pure evaluation for equal-workload ROS angle and tactile benchmarks."""

from __future__ import annotations

import math
from collections.abc import Iterable
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
    client_id: int
    user_sequence: int
    publisher_at: float | None
    driver_received_at: float | None
    transition_at: float | None
    stage: int
    ack_support: int


@dataclass(frozen=True, slots=True)
class LatencyStats:
    sample_count: int
    p50_ms: float | None
    p95_ms: float | None
    p99_ms: float | None


@dataclass(slots=True)
class _CommandRecord:
    publisher_at: float | None
    driver_received_at: float | None
    ack_support: int
    transitions: dict[int, float | None]


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
    client_id: int
    requested_angle_hz: float
    requested_tactile_hz: float
    angle_feedback_policy: str
    started_at: float
    ended_at: float
    sent_at: tuple[float, ...]
    state_events: tuple[StateEvent, ...]
    command_events: tuple[CommandEvent, ...]
    start_diagnostic: DiagnosticObservation | None
    measurement_end_diagnostic: DiagnosticObservation | None
    end_diagnostic: DiagnosticObservation | None


@dataclass(frozen=True, slots=True)
class BenchmarkResult:
    requested_angle_hz: float
    requested_tactile_hz: float
    passed: bool
    command_publish_hz: float
    command_received_hz: float
    command_accepted_hz: float
    command_terminal_hz: float
    command_dispatch_hz: float
    angle_feedback_hz: float | None
    angle_command_feedback_hz: float | None
    angle_read_completion_hz: float | None
    angle_cache_update_hz: float | None
    angle_ros_publish_hz: float | None
    tactile_read_completion_hz: float | None
    tactile_cache_update_hz: float | None
    tactile_ros_publish_hz: float | None
    command_interval_p95_ms: float | None
    publish_to_receive_latency: LatencyStats
    receive_to_accept_latency: LatencyStats
    accept_to_dispatch_latency: LatencyStats
    publish_to_dispatch_latency: LatencyStats
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
    if type(measurements.client_id) is not int or measurements.client_id <= 0:
        raise ValueError("benchmark client_id must be a positive int")
    feedback_policy = measurements.angle_feedback_policy
    if feedback_policy not in {
        "query_only",
        "command_feedback_only",
        "adaptive",
    }:
        raise ValueError("benchmark angle feedback policy is invalid")
    publish_hz = len(measurements.sent_at) / duration
    own_events = tuple(
        event
        for event in measurements.command_events
        if event.client_id == measurements.client_id
    )
    records = tuple(_command_records(own_events).values())
    received = tuple(
        record for record in records if record.driver_received_at is not None
    )
    accepted = tuple(
        record
        for record in records
        if HandCommandStatus.STAGE_ACCEPTED in record.transitions
    )
    dispatched = tuple(
        record
        for record in records
        if HandCommandStatus.STAGE_DISPATCHED in record.transitions
    )
    terminal = tuple(record for record in records if _is_protocol_terminal(record))
    received_hz = len(received) / duration
    accepted_hz = len(accepted) / duration
    dispatched_hz = len(dispatched) / duration
    terminal_hz = len(terminal) / duration
    failed = sum(
        HandCommandStatus.STAGE_FAILED in record.transitions for record in records
    )
    superseded = sum(
        HandCommandStatus.STAGE_SUPERSEDED in record.transitions for record in records
    )
    publish_to_receive, invalid_publish_to_receive = _latency_stats(
        (record.publisher_at, record.driver_received_at) for record in received
    )
    receive_to_accept, invalid_receive_to_accept = _latency_stats(
        (
            record.driver_received_at,
            record.transitions[HandCommandStatus.STAGE_ACCEPTED],
        )
        for record in accepted
    )
    accept_to_dispatch, invalid_accept_to_dispatch = _latency_stats(
        (
            record.transitions.get(HandCommandStatus.STAGE_ACCEPTED),
            record.transitions[HandCommandStatus.STAGE_DISPATCHED],
        )
        for record in dispatched
    )
    publish_to_dispatch, invalid_publish_to_dispatch = _latency_stats(
        (
            record.publisher_at,
            record.transitions[HandCommandStatus.STAGE_DISPATCHED],
        )
        for record in dispatched
    )
    angle_cache_hz, angle_publish_hz, angle_generation_changed = _state_rates(
        measurements.state_events,
        "angles",
    )
    tactile_cache_hz, tactile_publish_hz, tactile_generation_changed = _state_rates(
        measurements.state_events,
        "tactile",
    )
    angle_read_hz = _diagnostic_counter_rate(
        measurements.start_diagnostic,
        measurements.measurement_end_diagnostic,
        "channel_angles_completed_reads",
    )
    angle_command_feedback_hz = _diagnostic_counter_rate(
        measurements.start_diagnostic,
        measurements.measurement_end_diagnostic,
        "channel_angles_command_feedback_samples",
    )
    angle_feedback_hz = _diagnostic_counter_rate(
        measurements.start_diagnostic,
        measurements.measurement_end_diagnostic,
        "channel_angles_feedback_samples",
    )
    tactile_read_hz = (
        _diagnostic_counter_rate(
            measurements.start_diagnostic,
            measurements.measurement_end_diagnostic,
            "channel_force_completed_reads",
        )
        if measurements.requested_tactile_hz > 0
        else None
    )
    reasons: list[str] = []
    minimum_angle = measurements.requested_angle_hz * minimum_ratio
    if publish_hz < minimum_angle:
        reasons.append(f"命令发布频率 {publish_hz:.2f} Hz 低于 {minimum_angle:.2f} Hz")
    if received_hz < minimum_angle:
        reasons.append(
            f"驱动收到命令频率 {received_hz:.2f} Hz 低于 {minimum_angle:.2f} Hz"
        )
    if accepted_hz < minimum_angle:
        reasons.append(
            f"SDK ACCEPTED 频率 {accepted_hz:.2f} Hz 低于 {minimum_angle:.2f} Hz"
        )
    if terminal_hz < minimum_angle:
        reasons.append(
            f"命令协议终态频率 {terminal_hz:.2f} Hz 低于 {minimum_angle:.2f} Hz"
        )
    if dispatched_hz < minimum_angle:
        reasons.append(
            f"命令实际下发频率 {dispatched_hz:.2f} Hz 低于 {minimum_angle:.2f} Hz"
        )
    _append_rate_failure(
        reasons,
        "角度有效反馈频率",
        angle_feedback_hz,
        minimum_angle,
    )
    if feedback_policy == "query_only":
        _append_rate_failure(
            reasons,
            "角度主动查询完成频率",
            angle_read_hz,
            minimum_angle,
        )
    elif feedback_policy == "command_feedback_only":
        _append_rate_failure(
            reasons,
            "角度命令回显频率",
            angle_command_feedback_hz,
            minimum_angle,
        )
    _append_rate_failure(
        reasons,
        "角度缓存更新频率",
        angle_cache_hz,
        minimum_angle,
    )
    _append_rate_failure(
        reasons,
        "角度 ROS 发布频率",
        angle_publish_hz,
        minimum_angle,
    )
    if measurements.requested_tactile_hz > 0:
        minimum_tactile = measurements.requested_tactile_hz * minimum_ratio
        _append_rate_failure(
            reasons,
            "完整触觉读取完成频率",
            tactile_read_hz,
            minimum_tactile,
        )
        _append_rate_failure(
            reasons,
            "完整触觉缓存更新频率",
            tactile_cache_hz,
            minimum_tactile,
        )
        _append_rate_failure(
            reasons,
            "完整触觉 ROS 发布频率",
            tactile_publish_hz,
            minimum_tactile,
        )
    if angle_generation_changed or tactile_generation_changed:
        reasons.append("测试期间连接代次发生变化")
    if failed:
        reasons.append(f"存在 {failed} 个 FAILED 命令")
    for label, invalid in (
        ("发布到驱动接收", invalid_publish_to_receive),
        ("驱动接收到 SDK 接受", invalid_receive_to_accept),
        ("SDK 接受到实际下发", invalid_accept_to_dispatch),
        ("发布到实际下发", invalid_publish_to_dispatch),
    ):
        if invalid:
            reasons.append(f"{label}延迟存在 {invalid} 个缺失或倒序样本")
    _append_health_failures(measurements, reasons)
    return BenchmarkResult(
        requested_angle_hz=measurements.requested_angle_hz,
        requested_tactile_hz=measurements.requested_tactile_hz,
        passed=not reasons,
        command_publish_hz=publish_hz,
        command_received_hz=received_hz,
        command_accepted_hz=accepted_hz,
        command_terminal_hz=terminal_hz,
        command_dispatch_hz=dispatched_hz,
        angle_feedback_hz=angle_feedback_hz,
        angle_command_feedback_hz=angle_command_feedback_hz,
        angle_read_completion_hz=angle_read_hz,
        angle_cache_update_hz=angle_cache_hz,
        angle_ros_publish_hz=angle_publish_hz,
        tactile_read_completion_hz=tactile_read_hz,
        tactile_cache_update_hz=tactile_cache_hz,
        tactile_ros_publish_hz=tactile_publish_hz,
        command_interval_p95_ms=_interval_p95_ms(measurements.sent_at),
        publish_to_receive_latency=publish_to_receive,
        receive_to_accept_latency=receive_to_accept,
        accept_to_dispatch_latency=accept_to_dispatch,
        publish_to_dispatch_latency=publish_to_dispatch,
        failed_commands=failed,
        superseded_commands=superseded,
        reasons=tuple(reasons),
    )


def _is_protocol_terminal(record: _CommandRecord) -> bool:
    stages = record.transitions
    return bool(
        stages.keys()
        & {
            HandCommandStatus.STAGE_ACKNOWLEDGED,
            HandCommandStatus.STAGE_FAILED,
            HandCommandStatus.STAGE_SUPERSEDED,
        }
    ) or (
        HandCommandStatus.STAGE_DISPATCHED in stages
        and record.ack_support == HandCommandStatus.ACK_UNSUPPORTED
    )


def _command_records(
    events: tuple[CommandEvent, ...],
) -> dict[tuple[int, int], _CommandRecord]:
    records: dict[tuple[int, int], _CommandRecord] = {}
    for event in events:
        key = (event.client_id, event.user_sequence)
        record = records.get(key)
        if record is None:
            record = _CommandRecord(
                publisher_at=event.publisher_at,
                driver_received_at=event.driver_received_at,
                ack_support=event.ack_support,
                transitions={},
            )
            records[key] = record
        else:
            record.publisher_at = _earliest(record.publisher_at, event.publisher_at)
            record.driver_received_at = _earliest(
                record.driver_received_at,
                event.driver_received_at,
            )
        existing = record.transitions.get(event.stage)
        record.transitions[event.stage] = _earliest(existing, event.transition_at)
    return records


def _earliest(first: float | None, second: float | None) -> float | None:
    if first is None:
        return second
    if second is None:
        return first
    return min(first, second)


def _state_rates(
    events: tuple[StateEvent, ...],
    channel: str,
) -> tuple[float | None, float | None, bool]:
    channel_events = tuple(event for event in events if event.channel == channel)
    generations = {event.generation for event in channel_events}
    if len(generations) > 1:
        return None, None, True
    valid = tuple(event for event in channel_events if event.valid and not event.stale)
    if len(valid) < 2:
        return None, None, False
    elapsed = valid[-1].observed_at - valid[0].observed_at
    sequence_delta = valid[-1].sequence - valid[0].sequence
    if elapsed <= 0 or sequence_delta < 0:
        return None, None, False
    return sequence_delta / elapsed, (len(valid) - 1) / elapsed, False


def _diagnostic_counter_rate(
    start: DiagnosticObservation | None,
    end: DiagnosticObservation | None,
    key: str,
) -> float | None:
    if start is None or end is None:
        return None
    elapsed = end.observed_at - start.observed_at
    delta = _counter_delta(start, end, key)
    if elapsed <= 0 or delta is None:
        return None
    return delta / elapsed


def _append_rate_failure(
    reasons: list[str],
    label: str,
    value: float | None,
    minimum: float,
) -> None:
    if value is not None and value >= minimum:
        return
    actual = "unknown" if value is None else f"{value:.2f} Hz"
    reasons.append(f"{label} {actual} 低于 {minimum:.2f} Hz")


def _append_health_failures(
    measurements: BenchmarkMeasurements,
    reasons: list[str],
) -> None:
    start = measurements.start_diagnostic
    measurement_end = measurements.measurement_end_diagnostic
    end = measurements.end_diagnostic
    if start is None or measurement_end is None or end is None:
        reasons.append("缺少测试起止 health diagnostics")
        return
    generations = {
        observation.values.get("connection_generation")
        for observation in (start, measurement_end, end)
    }
    if len(generations) > 1:
        reasons.append("测试期间 health 连接代次发生变化")
    feedback_policy = measurement_end.values.get("channel_angles_feedback_policy")
    if feedback_policy != measurements.angle_feedback_policy:
        reasons.append(
            "角度反馈策略不匹配: "
            f"期望 {measurements.angle_feedback_policy}, 实际 {feedback_policy or 'unknown'}"
        )
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


def _latency_stats(
    timestamps: Iterable[tuple[float | None, float | None]],
) -> tuple[LatencyStats, int]:
    values_ms: list[float] = []
    invalid = 0
    for started_at, ended_at in timestamps:
        if started_at is None or ended_at is None or ended_at < started_at:
            invalid += 1
            continue
        values_ms.append((ended_at - started_at) * 1000)
    values_ms.sort()
    return (
        LatencyStats(
            sample_count=len(values_ms),
            p50_ms=_percentile(values_ms, 0.50),
            p95_ms=_percentile(values_ms, 0.95),
            p99_ms=_percentile(values_ms, 0.99),
        ),
        invalid,
    )


def _percentile(values: list[float], ratio: float) -> float | None:
    if not values:
        return None
    index = max(0, math.ceil(len(values) * ratio) - 1)
    return values[index]


__all__ = [
    "BenchmarkMeasurements",
    "BenchmarkResult",
    "CommandEvent",
    "DiagnosticObservation",
    "LatencyStats",
    "StateEvent",
    "evaluate_measurements",
]
