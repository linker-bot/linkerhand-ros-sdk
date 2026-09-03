import threading
from dataclasses import replace
from types import SimpleNamespace

import pytest
from linkerbot_hand import (
    AngleFeedbackPolicy,
    AngleSample,
    DataOrigin,
    ForceSample,
    SampleMeta,
    TactileFieldDescriptor,
    TactileFieldSample,
    TactileSensorMetadata,
    TactileSensorSample,
    TelemetrySemantic,
    get_declared_capabilities,
)
from linkerbot_hand_ros.state_publisher import SnapshotPublisher


class FailingPublisherNode:
    def __init__(self) -> None:
        self.plain: list[object] = []
        self.lifecycle: list[object] = []
        self.destroyed_plain: list[object] = []
        self.destroyed_lifecycle: list[object] = []

    def create_publisher(self, *_args, **_kwargs):
        publisher = object()
        self.plain.append(publisher)
        return publisher

    def create_lifecycle_publisher(self, *_args, **_kwargs):
        if len(self.lifecycle) == 3:
            raise RuntimeError("lifecycle publisher construction failed")
        publisher = object()
        self.lifecycle.append(publisher)
        return publisher

    def destroy_publisher(self, publisher) -> None:
        self.destroyed_plain.append(publisher)

    def destroy_lifecycle_publisher(self, publisher) -> None:
        self.destroyed_lifecycle.append(publisher)


def test_partial_construction_rolls_back_plain_and_lifecycle_publishers():
    node = FailingPublisherNode()
    capabilities = get_declared_capabilities("o6", side="right")

    with pytest.raises(RuntimeError, match="construction failed"):
        SnapshotPublisher(node, capabilities)

    assert set(node.destroyed_plain) == set(node.plain)
    assert set(node.destroyed_lifecycle) == set(node.lifecycle)


class FlakyPublisher:
    def __init__(self) -> None:
        self.calls = 0

    def publish(self, _message: object) -> None:
        self.calls += 1
        if self.calls == 1:
            raise RuntimeError("publication failed")


def test_failed_publication_does_not_suppress_retry():
    metadata = SampleMeta(
        sequence=1,
        collected_at=1.0,
        age_s=0.0,
        valid=True,
        stale=False,
        connection_generation=1,
        origin=DataOrigin.DEVICE_RESPONSE,
        within_declared_range=True,
    )
    snapshot = SimpleNamespace(
        angles=AngleSample(
            raw=(1, 2, 3, 4, 5, 6),
            percent=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
            physical_rad=(None, None, None, None, None, None),
            meta=metadata,
        ),
        measured_speeds=None,
        configured_speeds=None,
        configured_accelerations=None,
        currents=None,
        current_limits=None,
        effort_limits=None,
        torques=None,
        temperatures=None,
        faults=None,
        voltages=None,
        motion_times=None,
        force=None,
        stall_detection_times=None,
        stall_thresholds=None,
        stall_hold_currents=None,
        device_info=None,
    )
    publisher = SnapshotPublisher.__new__(SnapshotPublisher)
    publisher._lock = threading.RLock()
    publisher._published = {}
    publisher._angles_publisher = FlakyPublisher()
    publisher._telemetry_publishers = {}
    publisher._force_publisher = None
    publisher._tactile_publisher = None
    publisher._stall_publisher = None
    publisher._set_conversion_capabilities(
        get_declared_capabilities("o6", side="right")
    )

    with pytest.raises(RuntimeError, match="publication failed"):
        publisher.publish_snapshot(
            snapshot,
            ros_now_ns=1,
            angle_feedback_policy=AngleFeedbackPolicy.ADAPTIVE,
        )

    assert publisher._published == {}
    assert (
        publisher.publish_snapshot(
            snapshot,
            ros_now_ns=1,
            angle_feedback_policy=AngleFeedbackPolicy.ADAPTIVE,
        )
        == 1
    )
    assert (
        publisher.publish_snapshot(
            snapshot,
            ros_now_ns=1,
            angle_feedback_policy=AngleFeedbackPolicy.ADAPTIVE,
        )
        == 0
    )


class RecordingStatePublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


def _publisher_for_angle_test() -> SnapshotPublisher:
    publisher = SnapshotPublisher.__new__(SnapshotPublisher)
    publisher._lock = threading.RLock()
    publisher._published = {}
    publisher._angles_publisher = RecordingStatePublisher()
    publisher._telemetry_publishers = {}
    publisher._force_publisher = None
    publisher._tactile_publisher = None
    publisher._stall_publisher = None
    publisher._set_conversion_capabilities(
        get_declared_capabilities("o6", side="right")
    )
    return publisher


def _snapshot_with_angles(metadata: SampleMeta) -> SimpleNamespace:
    return SimpleNamespace(
        angles=AngleSample(
            raw=(1, 2, 3, 4, 5, 6),
            percent=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
            physical_rad=(None, None, None, None, None, None),
            meta=metadata,
        ),
        measured_speeds=None,
        configured_speeds=None,
        configured_accelerations=None,
        currents=None,
        current_limits=None,
        effort_limits=None,
        torques=None,
        temperatures=None,
        faults=None,
        voltages=None,
        motion_times=None,
        force=None,
        stall_detection_times=None,
        stall_thresholds=None,
        stall_hold_currents=None,
        device_info=None,
    )


@pytest.mark.parametrize(
    ("policy", "expected_publications"),
    (
        (AngleFeedbackPolicy.QUERY_ONLY, 0),
        (AngleFeedbackPolicy.COMMAND_FEEDBACK_ONLY, 1),
        (AngleFeedbackPolicy.ADAPTIVE, 1),
    ),
)
def test_angle_command_feedback_publication_matches_policy(
    policy,
    expected_publications,
):
    metadata = SampleMeta(
        sequence=1,
        collected_at=1.0,
        age_s=0.0,
        valid=True,
        stale=False,
        connection_generation=1,
        origin=DataOrigin.DEVICE_COMMAND_FEEDBACK,
        within_declared_range=True,
    )
    publisher = _publisher_for_angle_test()

    publications = publisher.publish_snapshot(
        _snapshot_with_angles(metadata),
        ros_now_ns=1,
        angle_feedback_policy=policy,
    )

    assert publications == expected_publications
    assert len(publisher._angles_publisher.messages) == expected_publications


def test_query_only_publishes_query_after_suppressing_command_feedback():
    command_feedback = SampleMeta(
        sequence=1,
        collected_at=1.0,
        age_s=0.0,
        valid=True,
        stale=False,
        connection_generation=1,
        origin=DataOrigin.DEVICE_COMMAND_FEEDBACK,
        within_declared_range=True,
    )
    query_response = replace(
        command_feedback,
        sequence=2,
        origin=DataOrigin.DEVICE_RESPONSE,
    )
    publisher = _publisher_for_angle_test()

    assert (
        publisher.publish_snapshot(
            _snapshot_with_angles(command_feedback),
            ros_now_ns=1,
            angle_feedback_policy=AngleFeedbackPolicy.QUERY_ONLY,
        )
        == 0
    )
    assert (
        publisher.publish_snapshot(
            _snapshot_with_angles(query_response),
            ros_now_ns=2,
            angle_feedback_policy=AngleFeedbackPolicy.QUERY_ONLY,
        )
        == 1
    )
    assert len(publisher._angles_publisher.messages) == 1


class RecordingCapabilityPublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class BlockingStatePublisher(RecordingStatePublisher):
    def __init__(self) -> None:
        super().__init__()
        self.started = threading.Event()
        self.release = threading.Event()

    def publish(self, message: object) -> None:
        self.started.set()
        assert self.release.wait(timeout=1.0)
        super().publish(message)


def test_capability_update_waits_for_inflight_reusable_state_message():
    metadata = SampleMeta(
        sequence=1,
        collected_at=1.0,
        age_s=0.0,
        valid=True,
        stale=False,
        connection_generation=1,
        origin=DataOrigin.DEVICE_RESPONSE,
        within_declared_range=True,
    )
    publisher = _publisher_for_angle_test()
    blocking = BlockingStatePublisher()
    publisher._angles_publisher = blocking
    capabilities = get_declared_capabilities("o6", side="right")
    publisher._capabilities_publisher = RecordingCapabilityPublisher()
    publisher._published_capabilities = capabilities
    state_thread = threading.Thread(
        target=lambda: publisher.publish_snapshot(
            _snapshot_with_angles(metadata),
            ros_now_ns=1,
            angle_feedback_policy=AngleFeedbackPolicy.ADAPTIVE,
        )
    )
    capability_updated = threading.Event()
    capability_thread = threading.Thread(
        target=lambda: (
            publisher.publish_capabilities(capabilities),
            capability_updated.set(),
        )
    )
    try:
        state_thread.start()
        assert blocking.started.wait(timeout=1.0)
        capability_thread.start()
        assert not capability_updated.wait(timeout=0.05)
        blocking.release.set()
        state_thread.join(timeout=1.0)
        capability_thread.join(timeout=1.0)
    finally:
        blocking.release.set()
        state_thread.join(timeout=1.0)
        capability_thread.join(timeout=1.0)

    assert not state_thread.is_alive()
    assert not capability_thread.is_alive()
    assert capability_updated.is_set()


def test_capability_publisher_emits_runtime_firmware_degradation_once():
    capabilities = get_declared_capabilities("o6", side="right")
    verified = replace(
        capabilities,
        identity=replace(capabilities.identity, verified=True, firmware="1.1.2"),
    )
    degraded = replace(
        verified,
        firmware_unsupported_telemetry=(TelemetrySemantic.FORCE,),
    )
    publisher = SnapshotPublisher.__new__(SnapshotPublisher)
    publisher._lock = threading.RLock()
    publisher._capabilities_publisher = RecordingCapabilityPublisher()
    publisher._published_capabilities = None

    assert publisher.publish_capabilities_if_changed(verified)
    assert not publisher.publish_capabilities_if_changed(verified)
    assert publisher.publish_capabilities_if_changed(degraded)
    assert not publisher.publish_capabilities_if_changed(degraded)
    assert len(publisher._capabilities_publisher.messages) == 2
    assert publisher._capabilities_publisher.messages[
        -1
    ].firmware_unsupported_telemetry == ["force"]


def _o30_force_sample(metadata: SampleMeta) -> ForceSample:
    descriptors = (
        TactileFieldDescriptor(
            semantic="FnS",
            data_type="U16",
            element_count=1,
            byte_offset=0,
            byte_length=2,
        ),
        TactileFieldDescriptor(
            semantic="Fn",
            data_type="U8",
            element_count=70,
            byte_offset=2,
            byte_length=70,
        ),
    )
    sensors = []
    for index, name in enumerate(
        ("thumb", "index", "middle", "ring", "pinky"), start=1
    ):
        normal_force = tuple((index + cell) & 0xFF for cell in range(70))
        sensors.append(
            TactileSensorSample(
                metadata=TactileSensorMetadata(
                    selected_sensor=index,
                    sensor_name=name,
                    sensor_type="TSSP_JZG",
                    data_unit="g",
                    data_range=255,
                    maximum_rows=10,
                    maximum_columns=7,
                    requested_rows=10,
                    requested_columns=7,
                    valid_data_position_mode=False,
                    rectangle_fill=False,
                    descriptor_text="FnS_U16_1;Fn_U8_70",
                    field_descriptors=descriptors,
                    total_data_length=72,
                ),
                raw_data=(index * 3).to_bytes(2, "little") + bytes(normal_force),
                fields=(
                    TactileFieldSample(
                        semantic="FnS",
                        data_type="U16",
                        element_count=1,
                        byte_offset=0,
                        byte_length=2,
                        values=(index * 3,),
                    ),
                    TactileFieldSample(
                        semantic="Fn",
                        data_type="U8",
                        element_count=70,
                        byte_offset=2,
                        byte_length=70,
                        values=normal_force,
                    ),
                ),
            )
        )
    return ForceSample(raw=(), tactile=(), sensors=tuple(sensors), meta=metadata)


def _empty_snapshot(**overrides: object) -> SimpleNamespace:
    fields = {
        name: None
        for name in (
            "angles",
            "measured_speeds",
            "configured_speeds",
            "configured_accelerations",
            "currents",
            "current_limits",
            "effort_limits",
            "torques",
            "temperatures",
            "faults",
            "voltages",
            "motion_times",
            "force",
            "stall_detection_times",
            "stall_thresholds",
            "stall_hold_currents",
            "device_info",
        )
    }
    fields.update(overrides)
    return SimpleNamespace(**fields)


def test_o30_descriptor_force_publishes_sensors_and_resultant_topics():
    class _PassingNode:
        def create_publisher(self, *_args, **_kwargs):
            return RecordingStatePublisher()

        def create_lifecycle_publisher(self, *_args, **_kwargs):
            return RecordingStatePublisher()

    capabilities = get_declared_capabilities("o30", side="right")

    publisher = SnapshotPublisher(_PassingNode(), capabilities)

    assert publisher._tactile_sensor_publisher is not None
    assert publisher._force_resultant_publisher is not None
    assert publisher._force_publisher is None
    assert publisher._tactile_publisher is None

    # Swap in recording publishers to observe the force publish path.
    publisher._tactile_sensor_publisher = RecordingStatePublisher()
    publisher._force_resultant_publisher = RecordingStatePublisher()
    # This force-focused test drives a lightweight snapshot, so silence the
    # unrelated stall path that expects a full HandSnapshot.
    publisher._stall_publisher = None

    metadata = SampleMeta(
        sequence=4,
        collected_at=1.0,
        age_s=0.0,
        valid=True,
        stale=False,
        connection_generation=1,
        origin=DataOrigin.DEVICE_RESPONSE,
        within_declared_range=True,
    )
    snapshot = _empty_snapshot(force=_o30_force_sample(metadata))

    published = publisher.publish_snapshot(
        snapshot,
        ros_now_ns=1,
        angle_feedback_policy=AngleFeedbackPolicy.ADAPTIVE,
    )

    assert published == 2
    sensor_messages = publisher._tactile_sensor_publisher.messages
    resultant_messages = publisher._force_resultant_publisher.messages
    assert len(sensor_messages) == 1
    assert len(resultant_messages) == 1
    assert [item.finger_name for item in sensor_messages[0].sensors] == [
        "thumb",
        "index",
        "middle",
        "ring",
        "pinky",
    ]
    assert list(resultant_messages[0].normal_force_resultant) == [
        3.0,
        6.0,
        9.0,
        12.0,
        15.0,
    ]
