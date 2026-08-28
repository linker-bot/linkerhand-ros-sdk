"""Typed persistent client contract tests."""

from __future__ import annotations

import threading
from dataclasses import replace

import pytest
import rclpy
from linkerbot_hand import CommandSemantic, get_declared_capabilities
from linkerbot_hand_ros.client import HandCommandClient
from linkerbot_hand_ros.conversion import capabilities_to_message
from rclpy.node import Node

from linkerbot_hand_msgs.msg import HandCommandStatus


class RecordingPublisher:
    def __init__(self, subscriptions: int = 1) -> None:
        self.messages: list[object] = []
        self.subscriptions = subscriptions

    def publish(self, message: object) -> None:
        self.messages.append(message)

    def get_subscription_count(self) -> int:
        return self.subscriptions


class FailingConstructionNode:
    def __init__(self) -> None:
        self.created: list[object] = []
        self.destroyed: list[object] = []

    def create_publisher(self, *_args, **_kwargs):
        if len(self.created) == 2:
            raise RuntimeError("publisher construction failed")
        publisher = object()
        self.created.append(publisher)
        return publisher

    def destroy_publisher(self, publisher) -> None:
        self.destroyed.append(publisher)

    def destroy_subscription(self, _subscription) -> None:
        raise AssertionError("no subscription should exist after publisher failure")


@pytest.fixture
def client_rig():
    rclpy.init()
    node = Node("typed_hand_client_test")
    client = HandCommandClient(node)
    angle = RecordingPublisher()
    original_angle = client._angle_publisher
    client._angle_publisher = angle
    raw_originals = dict(client._raw_publishers)
    raw = {name: RecordingPublisher() for name in client._raw_publishers}
    client._raw_publishers.update(raw)
    capabilities = get_declared_capabilities("l6", side="left")
    capabilities = replace(
        capabilities,
        identity=replace(capabilities.identity, verified=True),
    )
    client._update_capabilities(capabilities_to_message(capabilities))
    try:
        yield client, angle, raw
    finally:
        client._angle_publisher = original_angle
        client._raw_publishers.update(raw_originals)
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


def test_percent_and_raw_angles_use_one_typed_topic(client_rig):
    client, angle, _raw = client_rig
    percent = client.set_percent_angles([100, 50, 100, 100, 100, 100])
    capabilities = get_declared_capabilities("l6", side="left")
    command = capabilities.command(CommandSemantic.ANGLES)
    assert command is not None
    raw_values = [command.range_for(index).minimum for index in range(6)]
    raw = client.set_angles(raw_values)

    assert percent.user_sequence == 1
    assert raw.user_sequence == 2
    assert percent.representation == "percent"
    assert raw.representation == "raw"
    assert angle.messages[0].representation == angle.messages[0].REPRESENTATION_PERCENT
    assert angle.messages[1].representation == angle.messages[1].REPRESENTATION_RAW


def test_client_validates_from_published_capability_not_model_table(client_rig):
    client, _angle, _raw = client_rig

    with pytest.raises(ValueError, match="0..100"):
        client.set_percent_angles([101] * 6)
    with pytest.raises(ValueError, match="integer protocol"):
        client.set_angles([0.5] * 6)
    with pytest.raises(RuntimeError, match="does not declare torques"):
        client.set_torques([0] * 6)
    with pytest.raises(TypeError, match="bool"):
        client.set_percent_angles([True, 50, 50, 50, 50, 50])


def test_raw_command_publication_uses_capability_channel_shape(client_rig):
    client, _angle, raw_publishers = client_rig
    capabilities = get_declared_capabilities("l6", side="left")
    command = capabilities.command(CommandSemantic.SPEEDS)
    assert command is not None
    values = [command.range_for(index).minimum for index in range(6)]

    publication = client.set_speeds(values)

    assert publication.semantic == "speeds"
    assert publication.representation == "raw"
    assert list(raw_publishers["speeds"].messages[0].raw) == values


def test_wait_for_terminal_status_uses_sdk_stage_not_mechanical_arrival(client_rig):
    client, _angle, _raw = client_rig
    publication = client.set_percent_angles([100] * 6)
    result: list[HandCommandStatus | None] = []
    waiter = threading.Thread(
        target=lambda: result.append(
            client.wait_for_terminal_status(publication.user_sequence, 0.5)
        )
    )
    waiter.start()
    client._update_status(
        HandCommandStatus(
            user_sequence=publication.user_sequence,
            command_id=9,
            family="angle_target",
            stage=HandCommandStatus.STAGE_DISPATCHED,
            ack_support=HandCommandStatus.ACK_UNSUPPORTED,
            sent_frames=1,
        )
    )
    waiter.join(timeout=1)

    assert not waiter.is_alive()
    assert result[0] is not None
    assert result[0].stage == HandCommandStatus.STAGE_DISPATCHED


def test_readiness_requires_capability_and_driver_subscription(client_rig):
    client, angle, _raw = client_rig
    assert client.is_ready("angles")

    angle.subscriptions = 0
    assert not client.is_ready("angles")
    assert not client.is_ready("torques")
    with pytest.raises(RuntimeError, match="not active"):
        client.set_percent_angles([100] * 6)


def test_declared_capability_cannot_be_used_before_identity_verification(client_rig):
    client, _angle, _raw = client_rig
    client._update_capabilities(
        capabilities_to_message(get_declared_capabilities("l6", side="left"))
    )

    assert not client.is_ready("angles")
    with pytest.raises(RuntimeError, match="identity-verified"):
        client.set_percent_angles([100] * 6)


def test_client_destroy_is_idempotent_and_rejects_further_use(client_rig):
    client, _angle, _raw = client_rig
    client.destroy()
    client.destroy()

    with pytest.raises(RuntimeError, match="destroyed"):
        client.set_percent_angles([100] * 6)


def test_partial_publisher_construction_rolls_back_every_created_entity():
    node = FailingConstructionNode()

    with pytest.raises(RuntimeError, match="construction failed"):
        HandCommandClient(node)

    assert set(node.destroyed) == set(node.created)
