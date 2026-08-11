import threading
import time
from types import SimpleNamespace

import pytest

from linkerbot_hand_ros import client
from linkerbot_hand_ros.client import HandCommandClient
from linkerbot_hand_ros.qos import COMMAND_QOS, METADATA_QOS


class FakePublisher:
    def __init__(self, topic, qos):
        self.topic = topic
        self.qos = qos
        self.messages = []
        self.subscription_count = 1
        self.acknowledged = True

    def get_subscription_count(self):
        return self.subscription_count

    def publish(self, message):
        self.messages.append(message)

    def wait_for_all_acked(self, timeout):
        self.timeout = timeout
        return self.acknowledged


class FakeSubscription:
    def __init__(self, topic, callback, qos):
        self.topic = topic
        self.callback = callback
        self.qos = qos


class FakeNode:
    def __init__(self):
        self.publishers = []
        self.subscriptions = []
        self.destroyed_publishers = []
        self.destroyed_subscriptions = []

    def create_publisher(self, _message_type, topic, qos):
        publisher = FakePublisher(topic, qos)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, _message_type, topic, callback, qos):
        subscription = FakeSubscription(topic, callback, qos)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_publisher(self, publisher):
        self.destroyed_publishers.append(publisher)

    def destroy_subscription(self, subscription):
        self.destroyed_subscriptions.append(subscription)

    def receive(self, topic, data):
        subscription = next(item for item in self.subscriptions if item.topic == topic)
        subscription.callback(SimpleNamespace(data=data))


@pytest.fixture
def ready_client():
    node = FakeNode()
    command_client = HandCommandClient(node)
    node.receive(
        "/linkerbot_hand/joint_names",
        "thumb_flex,thumb_abd,index,middle,ring,pinky",
    )
    node.receive("/linkerbot_hand/model", "l6")
    return node, command_client


def test_client_creates_persistent_topics_with_shared_qos():
    node = FakeNode()
    command_client = HandCommandClient(node, driver_name="/left/linkerbot_hand/")

    assert command_client.driver_name == "/left/linkerbot_hand"
    assert [publisher.topic for publisher in node.publishers] == [
        "/left/linkerbot_hand/command/angles",
        "/left/linkerbot_hand/command/speeds",
        "/left/linkerbot_hand/command/torques",
    ]
    assert all(publisher.qos is COMMAND_QOS for publisher in node.publishers)
    assert all(subscription.qos is METADATA_QOS for subscription in node.subscriptions)


def test_client_rolls_back_partially_created_ros_entities():
    node = FakeNode()
    original_create_subscription = node.create_subscription
    subscription_calls = 0

    def fail_second_subscription(*args, **kwargs):
        nonlocal subscription_calls
        subscription_calls += 1
        if subscription_calls == 2:
            raise RuntimeError("injected subscription creation failure")
        return original_create_subscription(*args, **kwargs)

    node.create_subscription = fail_second_subscription

    with pytest.raises(RuntimeError, match="injected subscription creation failure"):
        HandCommandClient(node)

    assert len(node.destroyed_publishers) == 3
    assert len(node.destroyed_subscriptions) == 1


def test_client_rejects_commands_until_driver_metadata_is_ready():
    command_client = HandCommandClient(FakeNode())

    assert not command_client.is_ready()
    with pytest.raises(RuntimeError, match="not ready"):
        command_client.send_angles([100] * 6)


def test_client_rejects_inconsistent_driver_metadata():
    node = FakeNode()
    command_client = HandCommandClient(node)
    node.receive("/linkerbot_hand/joint_names", "joint_1,joint_2")
    node.receive("/linkerbot_hand/model", "l6")

    assert not command_client.is_ready()
    with pytest.raises(RuntimeError, match="not ready"):
        command_client.send_angles([100] * 6)


def test_client_waits_for_metadata_from_executor_thread():
    node = FakeNode()
    command_client = HandCommandClient(node)

    def publish_metadata():
        time.sleep(0.01)
        node.receive(
            "/linkerbot_hand/joint_names",
            "thumb_flex,thumb_abd,index,middle,ring,pinky",
        )
        node.receive("/linkerbot_hand/model", "l6")

    worker = threading.Thread(target=publish_metadata)
    worker.start()
    try:
        assert command_client.wait_until_ready(0.5)
    finally:
        worker.join(timeout=1.0)
    assert not worker.is_alive()


@pytest.mark.parametrize("timeout_sec", (0.0, -1.0, float("nan"), float("inf"), True))
def test_client_rejects_invalid_timeouts(ready_client, timeout_sec):
    _, command_client = ready_client

    with pytest.raises(ValueError, match="finite and positive"):
        command_client.wait_until_ready(timeout_sec)
    with pytest.raises(ValueError, match="finite and positive"):
        command_client.send_angles([100] * 6, timeout_sec=timeout_sec)


def test_client_reuses_publisher_without_waiting_for_ack_by_default(ready_client):
    node, command_client = ready_client

    receipt = command_client.send_angles([100, 50, 100, 100, 100, 20])
    second_receipt = command_client.send_angles([100] * 6)

    assert receipt.sequence == 1
    assert second_receipt.sequence == 2
    assert receipt.dds_acknowledged is None
    assert receipt.dds_ack_latency_ms is None
    assert len(node.publishers) == 3
    assert len(node.publishers[0].messages) == 2
    assert list(node.publishers[0].messages[0].data) == [
        100.0,
        50.0,
        100.0,
        100.0,
        100.0,
        20.0,
    ]


def test_client_can_wait_for_dds_ack(ready_client, monkeypatch):
    node, command_client = ready_client
    timestamps = iter((1_000_000, 6_250_000))
    monkeypatch.setattr(client.time, "monotonic_ns", lambda: next(timestamps))

    receipt = command_client.send_angles(
        [100, 50, 100, 100, 100, 100],
        wait_for_dds_ack=True,
        timeout_sec=0.5,
    )

    assert receipt.dds_acknowledged is True
    assert receipt.dds_ack_latency_ms == 5.25
    assert node.publishers[0].timeout.nanoseconds == 500_000_000


def test_client_uses_model_specific_validation(ready_client):
    _, command_client = ready_client

    with pytest.raises(ValueError, match="requires 6 values"):
        command_client.send_angles([100] * 5)
    with pytest.raises(ValueError, match=r"\[0, 100\]"):
        command_client.send_angles([101] * 6)


def test_client_tracks_driver_accepted_values(ready_client):
    node, command_client = ready_client

    assert command_client.last_accepted("angles") is None
    node.receive(
        "/linkerbot_hand/command/accepted/angles",
        [100.0, 50.0, 100.0, 100.0, 100.0, 20.0],
    )

    assert command_client.last_accepted("angles") == (
        100.0,
        50.0,
        100.0,
        100.0,
        100.0,
        20.0,
    )


def test_client_destroy_is_idempotent(ready_client):
    node, command_client = ready_client

    command_client.destroy()
    command_client.destroy()

    assert len(node.destroyed_publishers) == 3
    assert len(node.destroyed_subscriptions) == 5
    with pytest.raises(RuntimeError, match="destroyed"):
        command_client.send_angles([100] * 6)
    with pytest.raises(RuntimeError, match="destroyed"):
        command_client.is_ready()
    with pytest.raises(RuntimeError, match="destroyed"):
        command_client.last_accepted("angles")
