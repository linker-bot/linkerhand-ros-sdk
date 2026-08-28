from dataclasses import replace

from diagnostic_msgs.msg import DiagnosticStatus
from linkerbot_hand import (
    ChannelMetrics,
    ConnectionState,
    DataAge,
    DataOrigin,
    ErrorInfo,
    HealthStatus,
    LifecycleState,
    TransportError,
)
from linkerbot_hand_ros.diagnostics import build_diagnostic_status


def _operational_health() -> HealthStatus:
    return replace(
        HealthStatus.initial(),
        lifecycle_state=LifecycleState.OPEN,
        connection_state=ConnectionState.CONNECTED,
        connected=True,
        polling_active=True,
        connection_generation=3,
        identity_verified=True,
        channels=(
            ChannelMetrics(
                channel="angles",
                requested_hz=100.0,
                achieved_hz=99.5,
                missed_deadlines=0,
                timeouts=0,
                completed_reads=199,
                data_age=DataAge(
                    channel="angles",
                    age_s=0.002,
                    valid=True,
                    stale=False,
                    sequence=17,
                    connection_generation=3,
                    origin=DataOrigin.DEVICE_RESPONSE,
                ),
            ),
        ),
    )


def test_operational_health_maps_to_ok_without_other_sdk_state() -> None:
    status = build_diagnostic_status(
        node_name="/linkerbot_hand",
        model="L6",
        active=True,
        health=_operational_health(),
    )

    assert status.level == DiagnosticStatus.OK
    assert status.message == "operational"
    values = {item.key: item.value for item in status.values}
    assert values["connection_generation"] == "3"
    assert values["channel_angles_sequence"] == "17"
    assert values["channel_angles_achieved_hz"] == "99.5"
    assert values["channel_angles_completed_reads"] == "199"
    assert values["channel_angles_origin"] == "device_response"


def test_repeated_sdk_errors_take_priority_over_stale_angles() -> None:
    health = _operational_health()
    angles = health.channels[0]
    health = replace(
        health,
        consecutive_errors=3,
        recent_error=ErrorInfo.from_exception(TransportError("bus offline")),
        channels=(
            replace(
                angles,
                data_age=replace(angles.data_age, stale=True),
            ),
        ),
    )

    status = build_diagnostic_status(
        node_name="/linkerbot_hand",
        model="L6",
        active=True,
        health=health,
    )

    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "repeated SDK I/O errors"
    values = {item.key: item.value for item in status.values}
    assert values["recent_error_code"] == "transport"
    assert values["recent_error_type"] == "TransportError"


def test_inactive_lifecycle_is_reported_without_requiring_a_connection() -> None:
    status = build_diagnostic_status(
        node_name="/linkerbot_hand",
        model="L6",
        active=False,
        health=HealthStatus.initial(),
    )

    assert status.level == DiagnosticStatus.WARN
    assert status.message == "lifecycle node is inactive"
