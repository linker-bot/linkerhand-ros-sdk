"""Convert verified SDK identity and version samples into ROS messages."""

from __future__ import annotations

from linkerbot_hand import DeviceInfo, ValidationError, VersionField

from linkerbot_hand_msgs import msg as ros_messages

from .capabilities import CAPABILITY_SCHEMA_REVISION
from .time import sample_metadata_to_message


def device_info_to_message(
    info: DeviceInfo,
    *,
    ros_now_ns: int,
) -> ros_messages.HandDeviceInfo:
    """Convert verified identity while retaining all wire-level version data."""

    if not isinstance(info, DeviceInfo):
        raise ValidationError("info must be DeviceInfo")
    raw_identity = [] if info.raw_identity is None else list(info.raw_identity)
    return ros_messages.HandDeviceInfo(
        model=info.model,
        side=info.side.value,
        has_serial_number=info.serial_number is not None,
        serial_number=info.serial_number or "",
        pcb_version=_version_to_message(info.pcb_version),
        firmware_version=_version_to_message(info.firmware_version),
        mechanical_version=_version_to_message(info.mechanical_version),
        has_product_code=info.product_code is not None,
        product_code=info.product_code or "",
        has_sensor_variant=info.sensor_variant is not None,
        sensor_variant=info.sensor_variant or "",
        raw_identity=raw_identity,
        capability_revision=CAPABILITY_SCHEMA_REVISION,
        meta=sample_metadata_to_message(info.meta, ros_now_ns=ros_now_ns),
    )


def _version_to_message(
    value: VersionField | None,
) -> ros_messages.VersionField:
    if value is None:
        return ros_messages.VersionField(available=False)
    raw_bytes: list[int] = []
    raw_text = ""
    if isinstance(value.raw, str):
        raw_text = value.raw
    else:
        raw_bytes = list(value.raw)
    has_semantic_version = value.major is not None
    return ros_messages.VersionField(
        available=True,
        raw_bytes=raw_bytes,
        raw_text=raw_text,
        text=value.text,
        has_semantic_version=has_semantic_version,
        major=0 if value.major is None else value.major,
        minor=0 if value.minor is None else value.minor,
        patch=0 if value.patch is None else value.patch,
    )


__all__ = ["device_info_to_message"]
