from dataclasses import replace
from math import prod

import pytest
from builtin_interfaces.msg import Time
from linkerbot_hand import (
    SUPPORTED_MODELS,
    AckSupport,
    AngleSample,
    CommandFamily,
    CommandHandle,
    ConfiguredAccelerationSample,
    ConfiguredSpeedSample,
    CurrentLimitSample,
    CurrentSample,
    DataOrigin,
    DeviceInfo,
    EffortLimitSample,
    FaultSample,
    ForceSample,
    HandDevice,
    HandSide,
    MeasuredSpeedSample,
    MotionTimeSample,
    SampleMeta,
    StallDetectionTimeSample,
    StallHoldCurrentSample,
    StallThresholdSample,
    TactileMatrix,
    TelemetrySemantic,
    TemperatureSample,
    TorqueSample,
    TransportError,
    VersionField,
    VoltageSample,
    get_declared_capabilities,
)
from linkerbot_hand_ros.conversion import (
    CAPABILITY_SCHEMA_REVISION,
    angle_sample_to_message,
    capabilities_to_message,
    command_status_to_message,
    device_info_to_message,
    force_sample_to_messages,
    sample_metadata_to_message,
    stall_protection_to_message,
    telemetry_sample_to_message,
)

from linkerbot_hand_msgs.msg import HandCommandStatus, SampleMetadata

ROS_NOW_NS = 20_000_000_000

_SAMPLE_TYPES = {
    TelemetrySemantic.MEASURED_SPEEDS: MeasuredSpeedSample,
    TelemetrySemantic.CONFIGURED_SPEEDS: ConfiguredSpeedSample,
    TelemetrySemantic.CONFIGURED_ACCELERATIONS: ConfiguredAccelerationSample,
    TelemetrySemantic.CURRENTS: CurrentSample,
    TelemetrySemantic.CURRENT_LIMITS: CurrentLimitSample,
    TelemetrySemantic.EFFORT_LIMITS: EffortLimitSample,
    TelemetrySemantic.TORQUES: TorqueSample,
    TelemetrySemantic.TEMPERATURES: TemperatureSample,
    TelemetrySemantic.FAULTS: FaultSample,
    TelemetrySemantic.VOLTAGES: VoltageSample,
    TelemetrySemantic.MOTION_TIMES: MotionTimeSample,
    TelemetrySemantic.STALL_DETECTION_TIMES: StallDetectionTimeSample,
    TelemetrySemantic.STALL_THRESHOLDS: StallThresholdSample,
    TelemetrySemantic.STALL_HOLD_CURRENTS: StallHoldCurrentSample,
}


def _capabilities(model, side="left"):
    return get_declared_capabilities(
        model,
        **({} if model.lower() == "l30" else {"side": side}),
    )


def _meta(sequence=7, age_s=0.25, origin=DataOrigin.DEVICE_RESPONSE):
    return SampleMeta(
        sequence=sequence,
        collected_at=100.0,
        age_s=age_s,
        valid=True,
        stale=False,
        connection_generation=3,
        origin=origin,
        within_declared_range=True,
    )


@pytest.mark.parametrize("model", SUPPORTED_MODELS)
def test_all_model_capabilities_project_without_a_ros_model_table(model):
    capabilities = _capabilities(model)

    message = capabilities_to_message(capabilities)

    assert message.revision == CAPABILITY_SCHEMA_REVISION
    assert message.identity.model == capabilities.model
    assert message.identity.side == capabilities.side.value
    assert len(message.joints) == len(capabilities.joints)
    assert [joint.name for joint in message.joints] == list(capabilities.joint_names)
    assert [command.semantic for command in message.commands] == [
        command.semantic.value for command in capabilities.commands
    ]
    assert all(
        len(command.raw_ranges) == len(command.channel_names)
        for command in message.commands
    )
    assert [telemetry.semantic for telemetry in message.telemetry] == [
        telemetry.semantic.value for telemetry in capabilities.telemetry
    ]
    for sdk_value, ros_value in zip(
        capabilities.telemetry,
        message.telemetry,
        strict=True,
    ):
        expected_ranges = 0 if sdk_value.raw_range is None else len(sdk_value.channels)
        assert len(ros_value.raw_ranges) == expected_ranges


@pytest.mark.parametrize("model", SUPPORTED_MODELS)
def test_all_public_raw_telemetry_channels_use_the_generic_projection(model):
    capabilities = _capabilities(model)

    converted = []
    for channel in capabilities.telemetry:
        sample_type = _SAMPLE_TYPES.get(channel.semantic)
        if sample_type is None:
            continue
        sample = sample_type(
            raw=tuple(0 for _ in range(prod(channel.shape))),
            meta=_meta(),
        )
        message = telemetry_sample_to_message(
            sample,
            channel,
            ros_now_ns=ROS_NOW_NS,
        )
        assert message.semantic == channel.semantic.value
        assert len(message.raw) == prod(channel.shape)
        assert message.device_measured is channel.device_measured
        assert message.device_queried is channel.device_queried
        converted.append(channel.semantic)

    expected = [
        channel.semantic
        for channel in capabilities.telemetry
        if channel.semantic in _SAMPLE_TYPES
    ]
    assert converted == expected


def test_capabilities_keep_percent_direction_units_and_stall_metadata():
    l6 = capabilities_to_message(_capabilities("L6"))
    o6 = capabilities_to_message(_capabilities("O6", side="right"))
    o30i = capabilities_to_message(_capabilities("O30i"))

    thumb = l6.joints[0]
    assert thumb.percent_mapping.supported
    assert thumb.percent_mapping.zero_label == "flexed"
    assert thumb.percent_mapping.hundred_label == "extended"
    assert thumb.raw_angle_range.unit == "count"
    assert l6.transport.nominal_bitrate == 1_000_000
    assert not l6.transport.has_data_bitrate
    assert o6.operations.save_parameters
    assert "save_parameters" in o6.operations.acknowledged_operations

    o6_stall = o6.stall_protection
    assert o6_stall.threshold.semantic == "stall_thresholds"
    assert o6_stall.detection_time.semantic == "stall_detection_times"
    assert o6_stall.hold_current.semantic == "stall_hold_currents"
    assert o6_stall.threshold.raw_scale_note == (
        "1000/255 mA/count; lock/stall trigger threshold"
    )
    assert o6_stall.detection_time.raw_scale_note == "10 ms/count"
    assert o6_stall.hold_current.unit_confidence == "conflicting"

    assert o30i.stall_protection.supported
    assert not o30i.stall_protection.safety_range_verified
    stall = o30i.stall_protection
    expected_semantics = (
        "stall_detection_times",
        "stall_thresholds",
        "stall_hold_currents",
    )
    for field, semantic in zip(
        (stall.detection_time, stall.threshold, stall.hold_current),
        expected_semantics,
        strict=True,
    ):
        assert field.semantic == semantic
        assert list(field.channel_names) == list(stall.channel_names)
        assert len(field.raw_ranges) == len(stall.channel_names)
        assert field.read_supported is stall.read_supported
        assert field.write_supported is stall.write_supported


def test_runtime_firmware_degradation_is_projected_without_removing_metadata():
    capabilities = _capabilities("O6", side="right")
    capabilities = replace(
        capabilities,
        firmware_unsupported_telemetry=(
            TelemetrySemantic.CONFIGURED_ACCELERATIONS,
            TelemetrySemantic.FORCE,
        ),
    )

    message = capabilities_to_message(capabilities)

    assert message.firmware_unsupported_telemetry == [
        "configured_accelerations",
        "force",
    ]
    assert {value.semantic for value in message.telemetry} >= {
        "configured_accelerations",
        "force",
    }


def test_sample_metadata_projects_cached_age_into_ros_time():
    message = sample_metadata_to_message(_meta(), ros_now_ns=ROS_NOW_NS)

    assert message.sequence == 7
    assert message.age_s == 0.25
    assert message.collected_at.sec == 19
    assert message.collected_at.nanosec == 750_000_000
    assert message.origin == SampleMetadata.ORIGIN_DEVICE_RESPONSE

    unavailable = sample_metadata_to_message(
        SampleMeta.unavailable(connection_generation=4),
        ros_now_ns=ROS_NOW_NS,
    )
    assert not unavailable.valid
    assert unavailable.collected_at.sec == 0
    assert unavailable.collected_at.nanosec == 0

    feedback = sample_metadata_to_message(
        _meta(origin=DataOrigin.DEVICE_COMMAND_FEEDBACK),
        ros_now_ns=ROS_NOW_NS,
    )
    assert feedback.origin == SampleMetadata.ORIGIN_DEVICE_COMMAND_FEEDBACK


def test_angle_message_never_conflates_raw_percent_and_physical_values():
    capabilities = _capabilities("L6")
    sample = AngleSample(
        raw=(255, 128, 64, 32, 16, 0),
        percent=(100.0, 50.2, None, 12.5, 6.25, 0.0),
        physical_rad=(None, 0.1, None, None, None, None),
        meta=_meta(),
    )

    message = angle_sample_to_message(
        sample,
        capabilities,
        ros_now_ns=ROS_NOW_NS,
    )

    assert list(message.raw) == [255.0, 128.0, 64.0, 32.0, 16.0, 0.0]
    assert list(message.percent_valid) == [True, True, False, True, True, True]
    assert message.percent[2] == 0.0
    assert list(message.physical_rad_valid) == [False, True, False, False, False, False]


def test_command_cache_telemetry_is_not_published_as_device_measurement():
    capabilities = _capabilities("L6")
    channel = capabilities.telemetry_channel(TelemetrySemantic.CONFIGURED_SPEEDS)
    assert channel is not None
    sample = ConfiguredSpeedSample(
        raw=(1, 2, 3, 4, 5, 6),
        meta=_meta(origin=DataOrigin.COMMAND_CACHE),
    )

    message = telemetry_sample_to_message(
        sample,
        channel,
        ros_now_ns=ROS_NOW_NS,
    )

    assert message.semantic == "configured_speeds"
    assert not message.device_measured
    assert not message.device_queried
    assert message.meta.origin == SampleMetadata.ORIGIN_COMMAND_CACHE


@pytest.mark.parametrize("online", [True, False, None])
def test_tactile_conversion_preserves_matrix_shape_and_online_state(online):
    capabilities = _capabilities("O20")
    channel = capabilities.telemetry_channel(TelemetrySemantic.FORCE)
    assert channel is not None
    matrices = tuple(
        TactileMatrix(
            name=name,
            rows=12,
            columns=6,
            raw=tuple(tuple(range(6)) for _ in range(12)),
            online=online,
        )
        for name in channel.channels
    )
    sample = ForceSample(raw=(), tactile=matrices, meta=_meta())

    messages = force_sample_to_messages(
        sample,
        channel,
        ros_now_ns=ROS_NOW_NS,
    )

    assert messages.telemetry is None
    assert messages.tactile is not None
    first = messages.tactile.fingers[0]
    assert (first.rows, first.columns, len(first.raw)) == (12, 6, 72)
    assert first.has_online is (online is not None)
    assert first.online is (False if online is None else online)


def test_device_info_preserves_raw_and_semantic_version_representations():
    info = DeviceInfo(
        model="O6",
        serial_number="730",
        pcb_version=VersionField(raw="04.1", text="04.1"),
        firmware_version=VersionField(
            raw=bytes((2, 4, 0)),
            text="2.4.0",
            major=2,
            minor=4,
            patch=0,
        ),
        mechanical_version=None,
        product_code="LHO6-04.1-730-R-B-1-D",
        sensor_variant="B",
        side=HandSide.RIGHT,
        raw_identity=bytes((0xC0, 0, 1, 2)),
        meta=_meta(),
    )

    message = device_info_to_message(info, ros_now_ns=ROS_NOW_NS)

    assert message.pcb_version.raw_text == "04.1"
    assert list(message.firmware_version.raw_bytes) == [2, 4, 0]
    assert message.firmware_version.has_semantic_version
    assert message.firmware_version.major == 2
    assert not message.mechanical_version.available
    assert list(message.raw_identity) == [0xC0, 0, 1, 2]


def test_stall_state_keeps_three_independent_samples_and_metadata():
    capabilities = _capabilities("O30i")
    initial = HandDevice(
        model="O30i",
        side="left",
        transport=object(),
    ).get_snapshot()
    count = len(capabilities.stall_protection.channels)
    snapshot = replace(
        initial,
        stall_detection_times=StallDetectionTimeSample(
            raw=tuple(1 for _ in range(count)),
            meta=_meta(sequence=1),
        ),
        stall_thresholds=StallThresholdSample(
            raw=tuple(2 for _ in range(count)),
            meta=_meta(sequence=2),
        ),
        stall_hold_currents=StallHoldCurrentSample(
            raw=tuple(3 for _ in range(count)),
            meta=_meta(sequence=3),
        ),
    )

    message = stall_protection_to_message(
        snapshot,
        capabilities,
        ros_now_ns=ROS_NOW_NS,
    )

    assert message is not None
    assert message.detection_times.meta.sequence == 1
    assert message.thresholds.meta.sequence == 2
    assert message.hold_currents.meta.sequence == 3
    assert set(message.detection_times.raw) == {1.0}
    assert set(message.thresholds.raw) == {2.0}
    assert set(message.hold_currents.raw) == {3.0}


def test_command_status_distinguishes_dispatch_from_protocol_ack():
    handle = CommandHandle(
        11,
        CommandFamily.ANGLE_TARGET,
        ack_support=AckSupport.UNSUPPORTED,
        accepted_at=100.0,
    )
    handle._mark_dispatched(timestamp=100.25, sent_frames=1)

    message = command_status_to_message(
        handle,
        client_id=17,
        user_sequence=9,
        publisher_stamp=Time(sec=18, nanosec=250),
        driver_received_at=100.1,
        monotonic_now=101.0,
        ros_now_ns=ROS_NOW_NS,
    )

    assert message.stage == HandCommandStatus.STAGE_DISPATCHED
    assert message.client_id == 17
    assert message.publisher_stamp == Time(sec=18, nanosec=250)
    assert message.driver_received_stamp.sec == 19
    assert message.driver_received_stamp.nanosec == 100_000_000
    assert message.stage != HandCommandStatus.STAGE_ACKNOWLEDGED
    assert message.ack_support == HandCommandStatus.ACK_UNSUPPORTED
    assert [item.stage for item in message.transitions] == [
        HandCommandStatus.STAGE_ACCEPTED,
        HandCommandStatus.STAGE_DISPATCHED,
    ]
    assert message.transitions[0].stamp.sec == 19


def test_command_failure_uses_stable_error_code_without_string_parsing():
    handle = CommandHandle(
        12,
        CommandFamily.DISCRETE_OPERATION,
        ack_support=AckSupport.SUPPORTED,
        accepted_at=100.0,
    )
    handle._mark_failed(
        TransportError("bus disconnected"),
        timestamp=100.1,
        sent_frames=0,
    )

    message = command_status_to_message(
        handle,
        client_id=0,
        user_sequence=0,
        publisher_stamp=Time(),
        driver_received_at=100.05,
        monotonic_now=100.2,
        ros_now_ns=ROS_NOW_NS,
    )

    assert message.stage == HandCommandStatus.STAGE_FAILED
    assert message.error_code == "transport"
    assert message.error_message == "bus disconnected"
