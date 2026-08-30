"""Convert immutable SDK state samples into explicit ROS state messages."""

from __future__ import annotations

from dataclasses import dataclass
from math import prod

from linkerbot_hand import (
    AngleSample,
    ConfiguredAccelerationSample,
    ConfiguredSpeedSample,
    CurrentLimitSample,
    CurrentSample,
    DeviceCapabilities,
    EffortLimitSample,
    FaultSample,
    ForceSample,
    MeasuredSpeedSample,
    MotionTimeSample,
    SampleMeta,
    StallDetectionTimeSample,
    StallHoldCurrentSample,
    StallThresholdSample,
    TelemetryCapability,
    TelemetrySemantic,
    TemperatureSample,
    TorqueSample,
    ValidationError,
    VoltageSample,
)

from linkerbot_hand_msgs import msg as ros_messages

from .time import sample_metadata_to_message, update_sample_metadata_message

RawVectorSample = (
    MeasuredSpeedSample
    | ConfiguredSpeedSample
    | ConfiguredAccelerationSample
    | CurrentSample
    | CurrentLimitSample
    | EffortLimitSample
    | TorqueSample
    | TemperatureSample
    | FaultSample
    | VoltageSample
    | MotionTimeSample
    | StallDetectionTimeSample
    | StallThresholdSample
    | StallHoldCurrentSample
)


@dataclass(frozen=True, slots=True)
class ForceMessages:
    """Scalar and matrix projections produced from one force transaction."""

    telemetry: ros_messages.HandTelemetry | None
    tactile: ros_messages.HandTactileState | None


def angle_sample_to_message(
    sample: AngleSample,
    capabilities: DeviceCapabilities,
    *,
    ros_now_ns: int,
) -> ros_messages.HandJointState:
    """Preserve raw, percent, and physical angle representations separately."""

    if not isinstance(sample, AngleSample):
        raise ValidationError("sample must be AngleSample")
    if not isinstance(capabilities, DeviceCapabilities):
        raise ValidationError("capabilities must be DeviceCapabilities")
    if len(sample.raw) != len(capabilities.joints):
        raise ValidationError("angle sample shape does not match capabilities")
    return ros_messages.HandJointState(
        joint_names=list(capabilities.joint_names),
        raw=[float(value) for value in sample.raw],
        percent=[0.0 if value is None else value for value in sample.percent],
        percent_valid=[value is not None for value in sample.percent],
        physical_rad=[0.0 if value is None else value for value in sample.physical_rad],
        physical_rad_valid=[value is not None for value in sample.physical_rad],
        meta=sample_metadata_to_message(sample.meta, ros_now_ns=ros_now_ns),
    )


def update_angle_message(
    message: ros_messages.HandJointState,
    sample: AngleSample,
    capabilities: DeviceCapabilities,
    *,
    ros_now_ns: int,
) -> None:
    """Update a capability-shaped angle message for the high-rate publish path."""

    if not isinstance(message, ros_messages.HandJointState):
        raise ValidationError("message must be HandJointState")
    if not isinstance(sample, AngleSample):
        raise ValidationError("sample must be AngleSample")
    if not isinstance(capabilities, DeviceCapabilities):
        raise ValidationError("capabilities must be DeviceCapabilities")
    joint_count = len(capabilities.joints)
    if len(sample.raw) != joint_count or len(message.raw) != joint_count:
        raise ValidationError("angle sample shape does not match reusable message")
    for index in range(joint_count):
        percent = sample.percent[index]
        physical_rad = sample.physical_rad[index]
        message.raw[index] = float(sample.raw[index])
        message.percent[index] = 0.0 if percent is None else percent
        message.percent_valid[index] = percent is not None
        message.physical_rad[index] = 0.0 if physical_rad is None else physical_rad
        message.physical_rad_valid[index] = physical_rad is not None
    update_sample_metadata_message(
        message.meta,
        sample.meta,
        ros_now_ns=ros_now_ns,
    )


def make_angle_message(
    capabilities: DeviceCapabilities,
) -> ros_messages.HandJointState:
    """Create one reusable angle message with immutable capability dimensions."""

    if not isinstance(capabilities, DeviceCapabilities):
        raise ValidationError("capabilities must be DeviceCapabilities")
    joint_count = len(capabilities.joints)
    return ros_messages.HandJointState(
        joint_names=list(capabilities.joint_names),
        raw=[0.0] * joint_count,
        percent=[0.0] * joint_count,
        percent_valid=[False] * joint_count,
        physical_rad=[0.0] * joint_count,
        physical_rad_valid=[False] * joint_count,
    )


def telemetry_sample_to_message(
    sample: RawVectorSample,
    capability: TelemetryCapability,
    *,
    ros_now_ns: int,
) -> ros_messages.HandTelemetry:
    """Convert one non-angle raw channel without inventing physical units."""

    if not isinstance(capability, TelemetryCapability):
        raise ValidationError("capability must be TelemetryCapability")
    if not isinstance(sample, _RAW_VECTOR_SAMPLE_TYPES):
        raise ValidationError("sample must be a raw vector SDK sample")
    if len(sample.raw) != prod(capability.shape):
        raise ValidationError(
            f"{capability.semantic.value} sample shape does not match capabilities"
        )
    return _raw_telemetry_message(
        semantic=capability.semantic.value,
        channel_names=capability.channels,
        raw=sample.raw,
        raw_unit=capability.raw_unit,
        raw_scale_note=capability.raw_scale_note,
        unit_confidence=capability.unit_confidence.value,
        device_measured=capability.device_measured,
        device_queried=capability.device_queried,
        metadata=sample.meta,
        ros_now_ns=ros_now_ns,
    )


def force_sample_to_messages(
    sample: ForceSample,
    capability: TelemetryCapability,
    *,
    ros_now_ns: int,
) -> ForceMessages:
    """Convert scalar force data and complete tactile matrices independently."""

    if not isinstance(sample, ForceSample):
        raise ValidationError("sample must be ForceSample")
    if (
        not isinstance(capability, TelemetryCapability)
        or capability.semantic is not TelemetrySemantic.FORCE
    ):
        raise ValidationError("capability must describe force telemetry")

    telemetry = None
    if sample.raw:
        if len(capability.shape) != 1 or len(sample.raw) != prod(capability.shape):
            raise ValidationError("scalar force shape does not match capabilities")
        telemetry = _raw_telemetry_message(
            semantic=capability.semantic.value,
            channel_names=capability.channels,
            raw=sample.raw,
            raw_unit=capability.raw_unit,
            raw_scale_note=capability.raw_scale_note,
            unit_confidence=capability.unit_confidence.value,
            device_measured=capability.device_measured,
            device_queried=capability.device_queried,
            metadata=sample.meta,
            ros_now_ns=ros_now_ns,
        )

    tactile = None
    if sample.tactile:
        if len(capability.shape) != 3:
            raise ValidationError("tactile capability shape must have three dimensions")
        finger_count, rows, columns = capability.shape
        if len(sample.tactile) != finger_count or any(
            matrix.rows != rows or matrix.columns != columns
            for matrix in sample.tactile
        ):
            raise ValidationError("tactile sample shape does not match capabilities")
        if tuple(matrix.name for matrix in sample.tactile) != capability.channels:
            raise ValidationError("tactile sample order does not match capabilities")
        tactile = ros_messages.HandTactileState(
            fingers=[
                ros_messages.TactileMatrix(
                    finger_name=matrix.name,
                    rows=matrix.rows,
                    columns=matrix.columns,
                    raw=[float(value) for row in matrix.raw for value in row],
                    has_online=matrix.online is not None,
                    online=False if matrix.online is None else matrix.online,
                )
                for matrix in sample.tactile
            ],
            raw_unit=capability.raw_unit or "",
            raw_scale_note=capability.raw_scale_note or "",
            unit_confidence=capability.unit_confidence.value,
            meta=sample_metadata_to_message(sample.meta, ros_now_ns=ros_now_ns),
        )
    return ForceMessages(telemetry=telemetry, tactile=tactile)


def _raw_telemetry_message(
    *,
    semantic: str,
    channel_names: tuple[str, ...],
    raw: tuple[int | float, ...],
    raw_unit: str | None,
    raw_scale_note: str | None,
    unit_confidence: str,
    device_measured: bool,
    device_queried: bool,
    metadata: SampleMeta,
    ros_now_ns: int,
) -> ros_messages.HandTelemetry:
    return ros_messages.HandTelemetry(
        semantic=semantic,
        channel_names=list(channel_names),
        raw=[float(value) for value in raw],
        raw_unit=raw_unit or "",
        raw_scale_note=raw_scale_note or "",
        unit_confidence=unit_confidence,
        device_measured=device_measured,
        device_queried=device_queried,
        meta=sample_metadata_to_message(metadata, ros_now_ns=ros_now_ns),
    )


_RAW_VECTOR_SAMPLE_TYPES = (
    MeasuredSpeedSample,
    ConfiguredSpeedSample,
    ConfiguredAccelerationSample,
    CurrentSample,
    CurrentLimitSample,
    EffortLimitSample,
    TorqueSample,
    TemperatureSample,
    FaultSample,
    VoltageSample,
    MotionTimeSample,
    StallDetectionTimeSample,
    StallThresholdSample,
    StallHoldCurrentSample,
)

__all__ = [
    "ForceMessages",
    "RawVectorSample",
    "angle_sample_to_message",
    "force_sample_to_messages",
    "make_angle_message",
    "telemetry_sample_to_message",
    "update_angle_message",
]
