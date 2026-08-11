"""Pure mapping from SDK health and driver metrics to ROS diagnostics."""

from __future__ import annotations

from collections.abc import Mapping

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from linkerbot_hand import (
    CommandHandle,
    ConnectionState,
    DeviceCapabilities,
    HandSnapshot,
    HealthStatus,
)

from .metrics import DriverMetricsSnapshot


TRACKED_COMMAND_NAMES = ("angles", "speeds", "torques", "clear_faults")


def build_diagnostic_status(
    *,
    node_name: str,
    active: bool,
    capabilities: DeviceCapabilities,
    health: HealthStatus,
    snapshot: HandSnapshot,
    metrics: DriverMetricsSnapshot,
    latest_handles: Mapping[str, CommandHandle],
) -> DiagnosticStatus:
    """Build one deterministic diagnostic status without touching the device."""
    status = DiagnosticStatus()
    status.name = f"{node_name}: hand_driver"
    status.hardware_id = capabilities.model
    status.level, status.message = _level_and_message(
        active=active,
        health=health,
        snapshot=snapshot,
        metrics=metrics,
    )

    recent_error = health.recent_error
    values = [
        _value("model", capabilities.model),
        _value("firmware", capabilities.firmware or "unknown"),
        _value("side", capabilities.side.value),
        _value("joint_count", capabilities.joint_count),
        _value("percentage_convention", capabilities.percentage_convention),
        _value("connection_state", health.connection_state.value),
        _value("connected", health.connected),
        _value("connection_generation", health.connection_generation),
        _value("reconnect_count", health.reconnect_count),
        _value("consecutive_errors", health.consecutive_errors),
        _value("pending_commands", health.pending_commands),
        _value("superseded_commands", health.superseded_commands),
        _value("ros_state_publications", metrics.state_successes),
        _value("ros_state_errors", metrics.state_errors),
        _value("ros_commands_accepted", metrics.command_successes),
        _value("ros_commands_rejected", metrics.command_errors),
        _value("ros_accepted_feedback_errors", metrics.feedback_errors),
        _value("ros_recent_error", metrics.last_error),
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
    ]
    values.extend(
        _value(
            f"{age.sensor.value}_age_s",
            "never" if age.age_s is None else f"{age.age_s:.3f}",
        )
        for age in health.data_ages
    )
    for command in TRACKED_COMMAND_NAMES:
        handle = latest_handles.get(command)
        values.extend(
            (
                _value(
                    f"latest_{command}_command_id",
                    "none" if handle is None else handle.command_id,
                ),
                _value(
                    f"latest_{command}_stage",
                    "none" if handle is None else handle.stage.value,
                ),
                _value(
                    f"latest_{command}_error_code",
                    "none"
                    if handle is None or handle.error is None
                    else handle.error.code.value,
                ),
            )
        )
    status.values = values
    return status


def _level_and_message(
    *,
    active: bool,
    health: HealthStatus,
    snapshot: HandSnapshot,
    metrics: DriverMetricsSnapshot,
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
    if health.consecutive_errors >= 3:
        return DiagnosticStatus.ERROR, "repeated SDK I/O errors"
    if health.consecutive_errors > 0:
        return DiagnosticStatus.WARN, "latest SDK I/O operation failed"

    angles = snapshot.angles
    if angles is None or not angles.valid:
        return DiagnosticStatus.WARN, "waiting for first valid angle sample"
    if angles.stale:
        return DiagnosticStatus.WARN, "cached angle state is stale"
    if (
        metrics.latest_state_failed
        or metrics.latest_command_failed
        or metrics.latest_feedback_failed
    ):
        return DiagnosticStatus.WARN, "latest ROS driver operation failed"
    return DiagnosticStatus.OK, "operational"


def _value(key: str, value: object) -> KeyValue:
    return KeyValue(key=key, value=str(value))


__all__ = ["TRACKED_COMMAND_NAMES", "build_diagnostic_status"]
