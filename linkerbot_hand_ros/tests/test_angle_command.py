from types import SimpleNamespace

import pytest

from linkerbot_hand_ros.examples import angle_command


class FakeCommandClient:
    def __init__(self, acknowledged=True):
        self.acknowledged = acknowledged
        self.calls = []
        self.destroyed = False

    def send_angles(self, values, **kwargs):
        self.calls.append((values, kwargs))
        return SimpleNamespace(
            values=tuple(float(value) for value in values),
            dds_acknowledged=self.acknowledged,
        )

    def destroy(self):
        self.destroyed = True


class FakeNode:
    def __init__(self):
        self.logs = []
        self.destroyed = False

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append)

    def destroy_node(self):
        self.destroyed = True


def _run(monkeypatch, acknowledged=True):
    command_client = FakeCommandClient(acknowledged=acknowledged)
    node = FakeNode()
    monkeypatch.setattr(angle_command, "Node", lambda _name: node)
    monkeypatch.setattr(
        angle_command,
        "HandCommandClient",
        lambda _node, driver_name: command_client,
    )
    monkeypatch.setattr(angle_command, "spin_until_ready", lambda *_args: True)
    monkeypatch.setattr(angle_command.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(angle_command.rclpy, "try_shutdown", lambda: None)
    angle_command.main(args=["--values", "100", "50", "100", "100", "100", "20"])
    return command_client, node


def test_command_uses_persistent_client_and_waits_for_ack(monkeypatch):
    command_client, node = _run(monkeypatch)

    assert command_client.calls == [
        (
            [100.0, 50.0, 100.0, 100.0, 100.0, 20.0],
            {"wait_for_dds_ack": True, "timeout_sec": 5.0},
        )
    ]
    assert node.logs == ["已向 /linkerbot_hand 发送 6 个角度值"]
    assert command_client.destroyed
    assert node.destroyed


def test_command_reports_delivery_ack_timeout(monkeypatch):
    with pytest.raises(SystemExit, match="等待话题订阅者确认超时"):
        _run(monkeypatch, acknowledged=False)


def test_command_resolves_legacy_topic_to_driver_name(monkeypatch):
    captured = []
    command_client = FakeCommandClient()
    node = FakeNode()
    monkeypatch.setattr(angle_command, "Node", lambda _name: node)
    monkeypatch.setattr(
        angle_command,
        "HandCommandClient",
        lambda _node, driver_name: captured.append(driver_name) or command_client,
    )
    monkeypatch.setattr(angle_command, "spin_until_ready", lambda *_args: True)
    monkeypatch.setattr(angle_command.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(angle_command.rclpy, "try_shutdown", lambda: None)

    angle_command.main(
        args=[
            "--topic",
            "/left/linkerbot_hand/command/angles",
            "--values",
            "100",
            "50",
            "100",
            "100",
            "100",
            "100",
        ]
    )

    assert captured == ["/left/linkerbot_hand"]
