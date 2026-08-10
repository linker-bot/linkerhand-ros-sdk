from io import StringIO
from types import SimpleNamespace

import pytest

from linkerbot_hand_ros.adapter import validate_command
from linkerbot_hand_ros.examples import angle_controller


class FakeCommandClient:
    model = "l6"
    joint_names = (
        "thumb_flex",
        "thumb_abd",
        "index",
        "middle",
        "ring",
        "pinky",
    )

    def __init__(self, acknowledgements=(True, True)):
        self.acknowledgements = iter(acknowledgements)
        self.calls = []
        self.destroyed = False

    def is_ready(self):
        return True

    def send_angles(self, values, **kwargs):
        normalized = validate_command(self.model, values, "angles")
        self.calls.append((normalized, kwargs))
        return SimpleNamespace(
            values=tuple(normalized),
            dds_acknowledged=next(self.acknowledgements),
            dds_ack_latency_ms=5.25,
        )

    def destroy(self):
        self.destroyed = True


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def error(self, message):
        self.errors.append(message)


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()
        self.destroyed = False

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        self.destroyed = True


def _run(monkeypatch, input_text, acknowledgements=(True, True)):
    command_client = FakeCommandClient(acknowledgements=acknowledgements)
    node = FakeNode()
    monkeypatch.setattr(angle_controller, "Node", lambda _name: node)
    monkeypatch.setattr(
        angle_controller,
        "HandCommandClient",
        lambda _node, driver_name: command_client,
    )
    monkeypatch.setattr(angle_controller, "spin_until_ready", lambda *_args: True)
    monkeypatch.setattr(angle_controller.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(angle_controller.rclpy, "ok", lambda: True)
    monkeypatch.setattr(angle_controller.rclpy, "try_shutdown", lambda: None)
    monkeypatch.setattr(angle_controller.sys, "stdin", StringIO(input_text))
    angle_controller.main(args=[])
    return command_client, node


def test_controller_reuses_business_client_for_multiple_commands(monkeypatch):
    command_client, node = _run(
        monkeypatch,
        "100 50 100 100 100 20\n100,50,100,100,100,100\nquit\n",
    )

    assert [values for values, _kwargs in command_client.calls] == [
        [100.0, 50.0, 100.0, 100.0, 100.0, 20.0],
        [100.0, 50.0, 100.0, 100.0, 100.0, 100.0],
    ]
    assert all(
        kwargs == {"wait_for_dds_ack": True, "timeout_sec": 5.0}
        for _values, kwargs in command_client.calls
    )
    assert node.logger.infos[-2:] == [
        "已发送 6 个角度值，DDS 确认耗时 5.25 ms",
        "已发送 6 个角度值，DDS 确认耗时 5.25 ms",
    ]
    assert command_client.destroyed


@pytest.mark.parametrize(
    "line",
    (
        "100 50 100",
        "100 50 100 100 100 invalid",
        "100 50 100 100 100 101",
        "100 50 100 100 100 nan",
    ),
)
def test_controller_rejects_invalid_commands_and_stays_running(monkeypatch, line):
    command_client, node = _run(
        monkeypatch,
        f"{line}\n100 50 100 100 100 100\nquit\n",
    )

    assert len(command_client.calls) == 1
    assert len(node.logger.errors) == 1


def test_controller_reports_ack_timeout_and_accepts_next_input(monkeypatch):
    command_client, node = _run(
        monkeypatch,
        "100 50 100 100 100 20\n100 50 100 100 100 100\nquit\n",
        acknowledgements=(False, True),
    )

    assert len(command_client.calls) == 2
    assert node.logger.errors == ["等待话题订阅者确认超时: /linkerbot_hand"]
    assert node.logger.infos[-1] == "已发送 6 个角度值，DDS 确认耗时 5.25 ms"
