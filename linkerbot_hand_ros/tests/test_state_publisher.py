from types import SimpleNamespace

import pytest
from linkerbot_hand import DataOrigin, SampleMeta
from linkerbot_hand_ros.state_publisher import SnapshotPublisher

from linkerbot_hand_ros import state_publisher


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

    with pytest.raises(RuntimeError, match="construction failed"):
        SnapshotPublisher(node)

    assert set(node.destroyed_plain) == set(node.plain)
    assert set(node.destroyed_lifecycle) == set(node.lifecycle)


class FlakyPublisher:
    def __init__(self) -> None:
        self.calls = 0

    def publish(self, _message: object) -> None:
        self.calls += 1
        if self.calls == 1:
            raise RuntimeError("publication failed")


def test_failed_publication_does_not_suppress_retry(monkeypatch):
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
        angles=SimpleNamespace(meta=metadata),
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
    capabilities = SimpleNamespace(
        stall_protection=None,
        telemetry_channel=lambda _semantic: None,
    )
    publisher = SnapshotPublisher.__new__(SnapshotPublisher)
    publisher._published = {}
    publisher._angles_publisher = FlakyPublisher()
    monkeypatch.setattr(
        state_publisher, "angle_sample_to_message", lambda *_a, **_k: object()
    )

    with pytest.raises(RuntimeError, match="publication failed"):
        publisher.publish_snapshot(snapshot, capabilities, ros_now_ns=1)

    assert publisher._published == {}
    assert publisher.publish_snapshot(snapshot, capabilities, ros_now_ns=1) == 1
    assert publisher.publish_snapshot(snapshot, capabilities, ros_now_ns=1) == 0
