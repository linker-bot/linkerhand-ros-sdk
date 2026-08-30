import threading
from dataclasses import replace
from types import SimpleNamespace

import pytest
from linkerbot_hand import (
    AngleFeedbackPolicy,
    AngleSample,
    DataOrigin,
    SampleMeta,
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
