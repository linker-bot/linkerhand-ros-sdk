"""Pure mapping from zero-I/O SDK health to ROS diagnostics."""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from linkerbot_hand import ConnectionState, HealthStatus


def build_diagnostic_status(
    *,
    node_name: str,
    model: str,
    active: bool,
    health: HealthStatus,
) -> DiagnosticStatus:
    """Build one diagnostic status using only the cached health contract."""

    status = DiagnosticStatus()
    status.name = f"{node_name}: hand_driver"
    status.hardware_id = model
    status.level, status.message = _level_and_message(active=active, health=health)
    recent_error = health.recent_error
    status.values = [
        _value("model", model),
        _value("lifecycle_state", health.lifecycle_state.value),
        _value("connection_state", health.connection_state.value),
        _value("connected", health.connected),
        _value("polling_active", health.polling_active),
        _value("identity_verified", health.identity_verified),
        _value("connection_generation", health.connection_generation),
        _value("reconnect_count", health.reconnect_count),
        _value("consecutive_errors", health.consecutive_errors),
        _value("pending_commands", health.pending_commands),
        _value("superseded_commands", health.superseded_commands),
        _value("discrete_queue_depth", health.discrete_queue_depth),
        _value("last_successful_send_at", health.last_successful_send_at),
        _value("last_successful_receive_at", health.last_successful_receive_at),
        _value(
            "last_successful_transaction_at",
            health.last_successful_transaction_at,
        ),
        _value(
            "recent_error_code",
            "none" if recent_error is None else recent_error.code.value,
        ),
        _value(
            "recent_error_type",
            "none" if recent_error is None else recent_error.error_type,
        ),
        _value(
            "recent_error_message",
            "" if recent_error is None else recent_error.message,
        ),
        _value("transaction_count", health.transaction_latency.count),
        _value("transaction_p50_s", health.transaction_latency.p50_s),
        _value("transaction_p95_s", health.transaction_latency.p95_s),
        _value("transaction_p99_s", health.transaction_latency.p99_s),
        _value("transport_tx_frames", health.transport.tx_frames),
        _value("transport_rx_frames", health.transport.rx_frames),
        _value("transport_tx_bytes", health.transport.tx_bytes),
        _value("transport_rx_bytes", health.transport.rx_bytes),
        _value("transport_dropped_frames", health.transport.dropped_frames),
        _value("transport_route_misses", health.transport.route_misses),
        _value("transport_bus_errors", health.transport.bus_errors),
        _value("transport_pending_tx", health.transport.pending_tx),
        _value("transport_pending_rx", health.transport.pending_rx),
        _value(
            "transport_estimated_utilization",
            health.transport.estimated_utilization,
        ),
    ]
    for metrics in health.command_families:
        prefix = f"command_{metrics.family.value}"
        status.values.extend(
            (
                _value(f"{prefix}_pending", metrics.pending),
                _value(f"{prefix}_accepted", metrics.accepted),
                _value(f"{prefix}_dispatched", metrics.dispatched),
                _value(f"{prefix}_acknowledged", metrics.acknowledged),
                _value(f"{prefix}_failed", metrics.failed),
                _value(f"{prefix}_superseded", metrics.superseded),
            )
        )
    for channel in health.channels:
        prefix = f"channel_{channel.channel}"
        age = channel.data_age
        status.values.extend(
            (
                _value(f"{prefix}_requested_hz", channel.requested_hz),
                _value(f"{prefix}_achieved_hz", channel.achieved_hz),
                _value(f"{prefix}_completed_reads", channel.completed_reads),
                _value(
                    f"{prefix}_command_feedback_samples",
                    channel.command_feedback_samples,
                ),
                _value(f"{prefix}_feedback_samples", channel.feedback_samples),
                _value(
                    f"{prefix}_feedback_policy",
                    (
                        "none"
                        if channel.feedback_policy is None
                        else channel.feedback_policy.value
                    ),
                ),
                _value(f"{prefix}_missed_deadlines", channel.missed_deadlines),
                _value(f"{prefix}_timeouts", channel.timeouts),
                _value(f"{prefix}_age_s", age.age_s),
                _value(f"{prefix}_valid", age.valid),
                _value(f"{prefix}_stale", age.stale),
                _value(f"{prefix}_sequence", age.sequence),
                _value(f"{prefix}_generation", age.connection_generation),
                _value(f"{prefix}_origin", age.origin.value),
            )
        )
    return status


def _level_and_message(
    *,
    active: bool,
    health: HealthStatus,
) -> tuple[int, str]:
    if not active:
        return DiagnosticStatus.WARN, "lifecycle node is inactive"
    if health.connection_state is ConnectionState.RECONNECTING:
        return DiagnosticStatus.WARN, "SDK is reconnecting"
    if not health.connected:
        return (
            DiagnosticStatus.ERROR,
            f"SDK connection is {health.connection_state.value}",
        )
    if not health.identity_verified:
        return DiagnosticStatus.ERROR, "device identity is not verified"
    if health.consecutive_errors >= 3:
        return DiagnosticStatus.ERROR, "repeated SDK I/O errors"
    if health.consecutive_errors:
        return DiagnosticStatus.WARN, "latest SDK I/O operation failed"
    angles = next(
        (
            channel.data_age
            for channel in health.channels
            if channel.channel == "angles"
        ),
        None,
    )
    if angles is None or not angles.valid:
        return DiagnosticStatus.WARN, "waiting for first valid angle sample"
    if angles.stale:
        return DiagnosticStatus.WARN, "cached angle state is stale"
    return DiagnosticStatus.OK, "operational"


def _value(key: str, value: object) -> KeyValue:
    return KeyValue(key=key, value="none" if value is None else str(value))


__all__ = ["build_diagnostic_status"]
