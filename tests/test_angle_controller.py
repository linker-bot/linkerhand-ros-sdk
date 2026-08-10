from io import StringIO
from types import SimpleNamespace

import pytest

from linkerbot_hand_ros.examples import angle_controller


class FakePublisher:
    def __init__(self, acknowledgements=(True,)):
        self.acknowledgements = iter(acknowledgements)
        self.messages = []
        self.timeouts = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message)

    def wait_for_all_acked(self, timeout):
        self.timeouts.append(timeout)
        return next(self.acknowledgements)


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def error(self, message):
        self.errors.append(message)


class FakeNode:
    def __init__(self, publisher):
        self.publisher = publisher
        self.logger = FakeLogger()
        self.destroyed = False

    def create_publisher(self, *_args):
        return self.publisher

    def create_subscription(self, _type, _topic, callback, _depth):
        callback(SimpleNamespace(data="thumb_flex,thumb_abd,index,middle,ring,pinky"))
        return object()

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        self.destroyed = True


def _run(monkeypatch, input_text, acknowledgements=(True, True)):
    publisher = FakePublisher(acknowledgements=acknowledgements)
    node = FakeNode(publisher)
    timestamps = iter((1_000_000, 2_500_000, 3_000_000, 4_000_000))
    monkeypatch.setattr(angle_controller, "Node", lambda _name: node)
    monkeypatch.setattr(angle_controller.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(angle_controller.rclpy, "ok", lambda: True)
    monkeypatch.setattr(angle_controller.rclpy, "try_shutdown", lambda: None)
    monkeypatch.setattr(angle_controller.time, "monotonic_ns", lambda: next(timestamps))
    monkeypatch.setattr(angle_controller.sys, "stdin", StringIO(input_text))
    angle_controller.main(args=[])
    return publisher, node


def test_controller_reuses_publisher_for_multiple_commands(monkeypatch):
    publisher, node = _run(
        monkeypatch,
        "100 50 100 100 100 20\n100,50,100,100,100,100\nquit\n",
    )

    assert [list(message.data) for message in publisher.messages] == [
        [100.0, 50.0, 100.0, 100.0, 100.0, 20.0],
        [100.0, 50.0, 100.0, 100.0, 100.0, 100.0],
    ]
    assert len(publisher.timeouts) == 2
    assert node.logger.infos[-2:] == [
        "已发送 6 个角度值，DDS 确认耗时 1.50 ms",
        "已发送 6 个角度值，DDS 确认耗时 1.00 ms",
    ]
    assert node.destroyed


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
    publisher, node = _run(
        monkeypatch,
        f"{line}\n100 50 100 100 100 100\nquit\n",
    )

    assert len(publisher.messages) == 1
    assert len(node.logger.errors) == 1


def test_controller_reports_ack_timeout_and_accepts_next_input(monkeypatch):
    publisher, node = _run(
        monkeypatch,
        "100 50 100 100 100 20\n100 50 100 100 100 100\nquit\n",
        acknowledgements=(False, True),
    )

    assert len(publisher.messages) == 2
    assert node.logger.errors == [
        "等待话题订阅者确认超时: /linkerbot_hand/command/angles"
    ]
    assert node.logger.infos[-1] == "已发送 6 个角度值，DDS 确认耗时 1.00 ms"
