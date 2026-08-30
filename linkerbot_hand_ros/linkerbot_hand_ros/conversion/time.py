"""Project SDK monotonic timestamps into the active ROS clock."""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from linkerbot_hand import DataOrigin, SampleMeta, ValidationError

from linkerbot_hand_msgs.msg import SampleMetadata

_NANOSECONDS_PER_SECOND = 1_000_000_000
_ORIGIN_VALUES = {
    DataOrigin.NONE: SampleMetadata.ORIGIN_NONE,
    DataOrigin.DEVICE_RESPONSE: SampleMetadata.ORIGIN_DEVICE_RESPONSE,
    DataOrigin.DEVICE_REPORT: SampleMetadata.ORIGIN_DEVICE_REPORT,
    DataOrigin.DEVICE_CONFIG_RESPONSE: (SampleMetadata.ORIGIN_DEVICE_CONFIG_RESPONSE),
    DataOrigin.DEVICE_COMMAND_FEEDBACK: (SampleMetadata.ORIGIN_DEVICE_COMMAND_FEEDBACK),
    DataOrigin.COMMAND_CACHE: SampleMetadata.ORIGIN_COMMAND_CACHE,
}


def sample_metadata_to_message(
    metadata: SampleMeta,
    *,
    ros_now_ns: int,
) -> SampleMetadata:
    """Convert one sample's cached age without reading either clock."""

    if not isinstance(metadata, SampleMeta):
        raise ValidationError("metadata must be SampleMeta")
    _validate_ros_time(ros_now_ns)
    message = SampleMetadata()
    update_sample_metadata_message(
        message,
        metadata,
        ros_now_ns=ros_now_ns,
    )
    return message


def update_sample_metadata_message(
    message: SampleMetadata,
    metadata: SampleMeta,
    *,
    ros_now_ns: int,
) -> None:
    """Update a reusable ROS metadata message without allocating nested state."""

    if not isinstance(message, SampleMetadata):
        raise ValidationError("message must be SampleMetadata")
    if not isinstance(metadata, SampleMeta):
        raise ValidationError("metadata must be SampleMeta")
    _validate_ros_time(ros_now_ns)
    age_s = 0.0 if metadata.age_s is None else float(metadata.age_s)
    collected_ns = (
        max(0, ros_now_ns - round(age_s * _NANOSECONDS_PER_SECOND))
        if metadata.valid
        else 0
    )
    seconds, nanoseconds = divmod(collected_ns, _NANOSECONDS_PER_SECOND)
    message.collected_at.sec = seconds
    message.collected_at.nanosec = nanoseconds
    message.sequence = metadata.sequence
    message.age_s = age_s
    message.valid = metadata.valid
    message.stale = metadata.stale
    message.connection_generation = metadata.connection_generation
    message.origin = _ORIGIN_VALUES[metadata.origin]
    message.within_declared_range = metadata.within_declared_range


def monotonic_timestamp_to_message(
    timestamp: float,
    *,
    monotonic_now: float,
    ros_now_ns: int,
) -> Time:
    """Project one historical monotonic timestamp onto the current ROS clock."""

    for value, name in (
        (timestamp, "timestamp"),
        (monotonic_now, "monotonic_now"),
    ):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ValidationError(f"{name} must be finite numeric")
    _validate_ros_time(ros_now_ns)
    age_s = max(0.0, float(monotonic_now) - float(timestamp))
    return _time_from_nanoseconds(
        max(0, ros_now_ns - round(age_s * _NANOSECONDS_PER_SECOND))
    )


def _validate_ros_time(ros_now_ns: object) -> None:
    if type(ros_now_ns) is not int or ros_now_ns < 0:
        raise ValidationError("ros_now_ns must be a non-negative int")


def _time_from_nanoseconds(timestamp_ns: int) -> Time:
    seconds, nanoseconds = divmod(timestamp_ns, _NANOSECONDS_PER_SECOND)
    return Time(sec=seconds, nanosec=nanoseconds)


__all__ = [
    "monotonic_timestamp_to_message",
    "sample_metadata_to_message",
    "update_sample_metadata_message",
]
