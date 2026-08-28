"""Build one complete ROS stall-protection view from three SDK caches."""

from __future__ import annotations

from linkerbot_hand import (
    DeviceCapabilities,
    HandSnapshot,
    TelemetrySemantic,
    ValidationError,
)

from linkerbot_hand_msgs import msg as ros_messages

from .state import RawVectorSample, telemetry_sample_to_message


def stall_protection_to_message(
    snapshot: HandSnapshot,
    capabilities: DeviceCapabilities,
    *,
    ros_now_ns: int,
) -> ros_messages.HandStallProtectionState | None:
    """Return a complete three-field stall state once all caches are available."""

    if not isinstance(snapshot, HandSnapshot):
        raise ValidationError("snapshot must be HandSnapshot")
    if not isinstance(capabilities, DeviceCapabilities):
        raise ValidationError("capabilities must be DeviceCapabilities")
    stall = capabilities.stall_protection
    samples = (
        snapshot.stall_detection_times,
        snapshot.stall_thresholds,
        snapshot.stall_hold_currents,
    )
    if stall is None or any(sample is None for sample in samples):
        return None
    detection, thresholds, hold_currents = samples
    assert detection is not None
    assert thresholds is not None
    assert hold_currents is not None
    return ros_messages.HandStallProtectionState(
        channel_names=list(stall.channels),
        detection_times=_stall_field_message(
            detection,
            TelemetrySemantic.STALL_DETECTION_TIMES,
            capabilities,
            ros_now_ns=ros_now_ns,
        ),
        thresholds=_stall_field_message(
            thresholds,
            TelemetrySemantic.STALL_THRESHOLDS,
            capabilities,
            ros_now_ns=ros_now_ns,
        ),
        hold_currents=_stall_field_message(
            hold_currents,
            TelemetrySemantic.STALL_HOLD_CURRENTS,
            capabilities,
            ros_now_ns=ros_now_ns,
        ),
    )


def _stall_field_message(
    sample: RawVectorSample,
    semantic: TelemetrySemantic,
    capabilities: DeviceCapabilities,
    *,
    ros_now_ns: int,
) -> ros_messages.HandTelemetry:
    capability = capabilities.telemetry_channel(semantic)
    if capability is None:
        raise ValidationError(f"missing {semantic.value} telemetry capability")
    return telemetry_sample_to_message(
        sample,
        capability,
        ros_now_ns=ros_now_ns,
    )


__all__ = ["stall_protection_to_message"]
