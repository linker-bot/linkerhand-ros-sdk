from io import StringIO
from types import SimpleNamespace

from linkerbot_hand_ros.examples import angle_controller


class FakeCommandClient:
    model = "L6"
    joint_names = (
        "thumb_flex",
        "thumb_abd",
        "index",
        "middle",
        "ring",
        "pinky",
    )

    def __init__(self):
        self.calls = []
        self.destroyed = False

    def set_percent_angles(self, values):
        if len(values) != 6 or any(not 0 <= value <= 100 for value in values):
            raise ValueError("invalid percent angles")
        self.calls.append(("percent", values))
        return SimpleNamespace(user_sequence=len(self.calls), values=tuple(values))

    def set_angles(self, values):
        self.calls.append(("raw", values))
        return SimpleNamespace(user_sequence=len(self.calls), values=tuple(values))

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


def _run(monkeypatch, input_text, *, raw=False):
    client = FakeCommandClient()
    node = FakeNode()
    monkeypatch.setattr(angle_controller, "Node", lambda _name: node)
    monkeypatch.setattr(
        angle_controller,
        "HandCommandClient",
        lambda _node, driver_name: client,
    )
    monkeypatch.setattr(angle_controller, "spin_until_ready", lambda *_args: True)
    monkeypatch.setattr(angle_controller.rclpy, "init", lambda **_kwargs: None)
    monkeypatch.setattr(angle_controller.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        angle_controller.rclpy, "spin_once", lambda *_args, **_kwargs: None
    )
    monkeypatch.setattr(angle_controller.rclpy, "try_shutdown", lambda: None)
    monkeypatch.setattr(angle_controller.sys, "stdin", StringIO(input_text))
    angle_controller.main(args=["--raw"] if raw else [])
    return client, node


def test_controller_reuses_one_typed_client(monkeypatch):
    client, node = _run(
        monkeypatch,
        "100 50 100 100 100 20\n100,50,100,100,100,100\nquit\n",
    )

    assert [kind for kind, _values in client.calls] == ["percent", "percent"]
    assert node.logger.infos[-2:] == [
        "已发布序号 1，共 6 个角度值",
        "已发布序号 2，共 6 个角度值",
    ]
    assert client.destroyed


def test_controller_rejects_invalid_percent_and_continues(monkeypatch):
    client, node = _run(
        monkeypatch,
        "100 50 100\n100 50 100 100 100 100\nquit\n",
    )

    assert len(client.calls) == 1
    assert node.logger.errors == ["invalid percent angles"]


def test_controller_raw_mode_uses_exact_protocol_method(monkeypatch):
    client, _node = _run(monkeypatch, "1 2 3 4 5 6\nquit\n", raw=True)
    assert client.calls == [("raw", [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])]
