"""Project the SDK capability source of truth into ROS messages."""

from __future__ import annotations

from linkerbot_hand import (
    CommandCapability,
    CommandSemantic,
    DeviceCapabilities,
    IdentityCapability,
    JointCapability,
    NumericRange,
    OperationCapabilities,
    PercentMapping,
    PhysicalAngleMapping,
    StallFieldCapability,
    StallProtectionCapability,
    TelemetryCapability,
    TelemetrySemantic,
    TransportCapability,
    ValidationError,
)

from linkerbot_hand_msgs import msg as ros_messages

CAPABILITY_SCHEMA_REVISION = "1.0"


def capabilities_to_message(
    capabilities: DeviceCapabilities,
) -> ros_messages.HandCapabilities:
    """Convert a complete declared or verified SDK capability profile."""

    if not isinstance(capabilities, DeviceCapabilities):
        raise ValidationError("capabilities must be DeviceCapabilities")
    angle_command = capabilities.command(CommandSemantic.ANGLES)
    angle_unit = "" if angle_command is None else (angle_command.raw_unit or "")
    return ros_messages.HandCapabilities(
        revision=CAPABILITY_SCHEMA_REVISION,
        identity=_identity_to_message(capabilities.identity),
        joints=[
            _joint_to_message(joint, raw_unit=angle_unit)
            for joint in capabilities.joints
        ],
        commands=[_command_to_message(value) for value in capabilities.commands],
        telemetry=[_telemetry_to_message(value) for value in capabilities.telemetry],
        stall_protection=_stall_to_message(capabilities.stall_protection),
        transport=_transport_to_message(capabilities.transport),
        operations=_operations_to_message(capabilities.operations),
    )


def _identity_to_message(
    value: IdentityCapability,
) -> ros_messages.IdentityCapability:
    return ros_messages.IdentityCapability(
        model=value.model,
        side=value.side.value,
        verified=value.verified,
        has_firmware=value.firmware is not None,
        firmware=value.firmware or "",
        has_hardware_revision=value.hardware_revision is not None,
        hardware_revision=value.hardware_revision or "",
        has_mechanical_revision=value.mechanical_revision is not None,
        mechanical_revision=value.mechanical_revision or "",
        has_sensor_variant=value.sensor_variant is not None,
        sensor_variant=value.sensor_variant or "",
        has_serial_number=value.serial_number is not None,
        serial_number=value.serial_number or "",
        has_product_code=value.product_code is not None,
        product_code=value.product_code or "",
        supported_fields=list(value.supported_fields),
    )


def _joint_to_message(
    value: JointCapability,
    *,
    raw_unit: str,
) -> ros_messages.JointCapability:
    return ros_messages.JointCapability(
        index=value.index,
        name=value.name,
        motion_type=value.motion_type.value,
        raw_angle_range=_range_to_message(value.raw_angle_range, unit=raw_unit),
        percent_mapping=_percent_to_message(value.percent_mapping),
        physical_angle_mapping=_physical_to_message(value.physical_angle_mapping),
        coupled_with=list(value.coupled_with),
    )


def _range_to_message(
    value: NumericRange,
    *,
    unit: str = "",
) -> ros_messages.NumericRange:
    return ros_messages.NumericRange(
        minimum=float(value.minimum),
        maximum=float(value.maximum),
        integer_only=value.integer_only,
        has_step=value.step is not None,
        step=0.0 if value.step is None else float(value.step),
        unit=unit,
    )


def _percent_to_message(
    value: PercentMapping | None,
) -> ros_messages.PercentMapping:
    if value is None:
        return ros_messages.PercentMapping(
            supported=False,
            confidence="unknown",
        )
    return ros_messages.PercentMapping(
        supported=True,
        raw_at_zero=float(value.raw_at_zero),
        raw_at_hundred=float(value.raw_at_hundred),
        zero_label=value.zero_label,
        hundred_label=value.hundred_label,
        confidence=value.confidence.value,
        quantization=value.quantization.value,
    )


def _physical_to_message(
    value: PhysicalAngleMapping | None,
) -> ros_messages.PhysicalAngleMapping:
    if value is None:
        return ros_messages.PhysicalAngleMapping(
            supported=False,
            confidence="unknown",
        )
    return ros_messages.PhysicalAngleMapping(
        supported=True,
        raw_at_first=float(value.raw_at_first),
        raw_at_second=float(value.raw_at_second),
        rad_at_first=float(value.radians_at_first),
        rad_at_second=float(value.radians_at_second),
        confidence=value.confidence.value,
    )


def _command_to_message(
    value: CommandCapability,
) -> ros_messages.CommandCapability:
    mappings = value.percent_mappings
    if mappings is None:
        mappings = tuple(None for _ in value.channels)
    return ros_messages.CommandCapability(
        semantic=value.semantic.value,
        channel_names=list(value.channels),
        raw_ranges=[
            _range_to_message(value.range_for(index), unit=value.raw_unit or "")
            for index in range(len(value.channels))
        ],
        raw_encoding=value.raw_encoding,
        has_raw_unit=value.raw_unit is not None,
        raw_unit=value.raw_unit or "",
        has_raw_scale_note=value.raw_scale_note is not None,
        raw_scale_note=value.raw_scale_note or "",
        unit_confidence=value.unit_confidence.value,
        ack_support=value.ack_support.value,
        percent_mappings=[_percent_to_message(item) for item in mappings],
    )


def _telemetry_to_message(
    value: TelemetryCapability,
) -> ros_messages.TelemetryCapability:
    ranges = (
        []
        if value.raw_range is None
        else [
            _range_to_message(value.raw_range, unit=value.raw_unit or "")
            for _ in value.channels
        ]
    )
    return ros_messages.TelemetryCapability(
        semantic=value.semantic.value,
        channel_names=list(value.channels),
        shape=list(value.shape),
        raw_ranges=ranges,
        raw_encoding=value.raw_encoding,
        has_raw_unit=value.raw_unit is not None,
        raw_unit=value.raw_unit or "",
        has_raw_scale_note=value.raw_scale_note is not None,
        raw_scale_note=value.raw_scale_note or "",
        unit_confidence=value.unit_confidence.value,
        read_mode=value.read_mode.value,
        recommended_hz=float(value.recommended_hz),
        has_verified_max_hz=value.verified_max_hz is not None,
        verified_max_hz=(
            0.0 if value.verified_max_hz is None else float(value.verified_max_hz)
        ),
        stale_after_s=float(value.stale_after_s),
        estimated_transaction_s=float(value.estimated_transaction_s),
        device_measured=value.device_measured,
        device_queried=value.device_queried,
    )


def _stall_field_to_message(
    value: StallFieldCapability,
    *,
    semantic: TelemetrySemantic,
    channels: tuple[str, ...],
    read_supported: bool,
    write_supported: bool,
) -> ros_messages.StallFieldCapability:
    return ros_messages.StallFieldCapability(
        semantic=semantic.value,
        channel_names=list(channels),
        raw_ranges=[
            _range_to_message(value.raw_range, unit=value.raw_unit or "")
            for _ in channels
        ],
        raw_encoding=value.raw_encoding,
        has_raw_unit=value.raw_unit is not None,
        raw_unit=value.raw_unit or "",
        has_raw_scale_note=value.raw_scale_note is not None,
        raw_scale_note=value.raw_scale_note or "",
        unit_confidence=value.unit_confidence.value,
        read_supported=read_supported,
        write_supported=write_supported,
    )


def _stall_to_message(
    value: StallProtectionCapability | None,
) -> ros_messages.StallProtectionCapability:
    if value is None:
        return ros_messages.StallProtectionCapability(supported=False)
    return ros_messages.StallProtectionCapability(
        supported=True,
        channel_names=list(value.channels),
        detection_time=_stall_field_to_message(
            value.detection_time,
            semantic=TelemetrySemantic.STALL_DETECTION_TIMES,
            channels=value.channels,
            read_supported=value.read_supported,
            write_supported=value.write_supported,
        ),
        threshold=_stall_field_to_message(
            value.threshold,
            semantic=TelemetrySemantic.STALL_THRESHOLDS,
            channels=value.channels,
            read_supported=value.read_supported,
            write_supported=value.write_supported,
        ),
        hold_current=_stall_field_to_message(
            value.hold_current,
            semantic=TelemetrySemantic.STALL_HOLD_CURRENTS,
            channels=value.channels,
            read_supported=value.read_supported,
            write_supported=value.write_supported,
        ),
        physical_semantics_confidence=(value.physical_semantics_confidence.value),
        safety_range_verified=value.safety_range_verified,
        read_supported=value.read_supported,
        write_supported=value.write_supported,
    )


def _transport_to_message(
    value: TransportCapability,
) -> ros_messages.TransportCapability:
    return ros_messages.TransportCapability(
        kind=value.kind.value,
        has_nominal_bitrate=value.nominal_bitrate is not None,
        nominal_bitrate=0 if value.nominal_bitrate is None else value.nominal_bitrate,
        has_data_bitrate=value.data_bitrate is not None,
        data_bitrate=0 if value.data_bitrate is None else value.data_bitrate,
        addressing_note=value.addressing_note,
    )


def _operations_to_message(
    value: OperationCapabilities,
) -> ros_messages.OperationCapabilities:
    return ros_messages.OperationCapabilities(
        read_device_info=value.read_device_info,
        clear_faults=value.clear_faults,
        enable=value.enable,
        disable=value.disable,
        calibrate=value.calibrate,
        read_stall_protection=value.read_stall_protection,
        write_stall_protection=value.write_stall_protection,
        acknowledged_operations=list(value.acknowledged_operations),
    )


__all__ = ["CAPABILITY_SCHEMA_REVISION", "capabilities_to_message"]
