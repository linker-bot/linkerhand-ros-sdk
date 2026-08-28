from types import SimpleNamespace

import pytest
from linkerbot_hand_ros.examples import angle_command

from linkerbot_hand_msgs.msg import HandCommandStatus


class FakeCommandClient:
    def __init__(self):
        self.calls = []
        self.destroyed = False

    def set_percent_angles(self, values):
        self.calls.append(("percent", values))
        return SimpleNamespace(user_sequence=3)

    def set_angles(self, values):
        self.calls.append(("raw", values))
        return SimpleNamespace(user_sequence=4)

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


def _run(monkeypatch, *, raw=False, stage=HandCommandStatus.STAGE_DISPATCHED):
    client = FakeCommandClient()
    node = FakeNode()
    monkeypatch.setattr(angle_command, "Node", lambda _name: node)
    monkeypatch.setattr(
        angle_command,
        "HandCommandClient",
        lambda _node, driver_name: client,
    )
    monkeypatch.setattr(angle_command, "spin_until_ready", lambda *_args: True)
    monkeypatch.setattr(
        angle_command,
        "spin_until_terminal",
        lambda *_args: HandCommandStatus(
            command_id=8,
            stage=stage,
            ack_support=HandCommandStatus.ACK_UNSUPPORTED,
            error_message="rejected" if stage == HandCommandStatus.STAGE_FAILED else "",
        ),
    )
    monkeypatch.setattr(angle_command.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(angle_command.rclpy, "try_shutdown", lambda: None)
    args = ["--values", "100", "50", "100", "100", "100", "20"]
    if raw:
        args.append("--raw")
    angle_command.main(args=args)
    return client, node


def test_command_uses_percent_contract_and_reports_sdk_stage(monkeypatch):
    client, node = _run(monkeypatch)

    assert client.calls == [("percent", [100.0, 50.0, 100.0, 100.0, 100.0, 20.0])]
    assert node.logs == [f"命令 8 已到达协议阶段 {HandCommandStatus.STAGE_DISPATCHED}"]
    assert client.destroyed
    assert node.destroyed


def test_command_selects_exact_protocol_raw_representation(monkeypatch):
    client, _node = _run(monkeypatch, raw=True)
    assert client.calls[0][0] == "raw"


def test_command_reports_sdk_failure(monkeypatch):
    with pytest.raises(SystemExit, match="rejected"):
        _run(monkeypatch, stage=HandCommandStatus.STAGE_FAILED)
