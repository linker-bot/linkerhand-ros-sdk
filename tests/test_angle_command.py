from types import SimpleNamespace

import pytest

from linkerbot_hand_ros.examples import angle_command


class FakePublisher:
    def __init__(self, acknowledged=True):
        self.acknowledged = acknowledged
        self.messages = []
        self.timeouts = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message)

    def wait_for_all_acked(self, timeout):
        self.timeouts.append(timeout)
        return self.acknowledged


class FakeNode:
    def __init__(self, publisher):
        self.publisher = publisher
        self.logs = []
        self.destroyed = False

    def create_publisher(self, *_args):
        return self.publisher

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append)

    def destroy_node(self):
        self.destroyed = True


def _run(monkeypatch, acknowledged=True):
    publisher = FakePublisher(acknowledged=acknowledged)
    node = FakeNode(publisher)
    monkeypatch.setattr(angle_command, "Node", lambda _name: node)
    monkeypatch.setattr(angle_command.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(angle_command.rclpy, "try_shutdown", lambda: None)
    angle_command.main(args=["--values", "100", "50", "100", "100", "100", "20"])
    return publisher, node


def test_command_waits_for_reliable_delivery_ack(monkeypatch):
    publisher, node = _run(monkeypatch)

    assert list(publisher.messages[0].data) == [100.0, 50.0, 100.0, 100.0, 100.0, 20.0]
    assert publisher.timeouts[0].nanoseconds == 5_000_000_000
    assert node.logs == [
        "已向 /linkerbot_hand/command/angles 发送 6 个角度值"
    ]
    assert node.destroyed


def test_command_reports_delivery_ack_timeout(monkeypatch):
    with pytest.raises(SystemExit, match="等待话题订阅者确认超时"):
        _run(monkeypatch, acknowledged=False)
