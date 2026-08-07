from enum import Flag
from types import SimpleNamespace

import pytest

from linkerbot_hand_ros.adapter import (
    HandAdapter,
    MODEL_SPECS,
    _as_float_list,
    create_sdk_hand,
)


class FakeManager:
    def __init__(self, attribute="angles"):
        self.attribute = attribute
        self.commands = []

    def set_angles(self, values):
        self.commands.append(values)

    def set_speeds(self, values):
        self.commands.append(values)

    def set_torques(self, values):
        self.commands.append(values)

    def get_blocking(self, timeout_ms):
        vector = SimpleNamespace(to_list=lambda: [25.0] * 6)
        return SimpleNamespace(**{self.attribute: vector}, timeout_ms=timeout_ms)


class FakeHand:
    def __init__(self):
        self.angle = FakeManager()
        self.speed = FakeManager("speeds")
        self.torque = FakeManager("torques")
        self.temperature = FakeManager("temperatures")
        self.closed = False

    def close(self):
        self.closed = True


@pytest.fixture
def hand_and_adapter():
    hand = FakeHand()

    def factory(*args, **kwargs):
        return hand

    return hand, HandAdapter("l6", hand_factory=factory)


def test_all_models_have_stable_joint_counts():
    assert {name: len(spec.joint_names) for name, spec in MODEL_SPECS.items()} == {
        "o6": 6,
        "l6": 6,
        "l20lite": 10,
        "l25": 16,
        "o20": 16,
        "o30i": 20,
        "l30": 17,
    }


def test_commands_and_readback_use_uniform_lists(hand_and_adapter):
    hand, adapter = hand_and_adapter
    adapter.set_angles([10] * 6)
    adapter.set_speeds([20] * 6)
    adapter.set_torques([30] * 6)
    assert hand.angle.commands == [[10.0] * 6]
    assert hand.speed.commands == [[20.0] * 6]
    assert hand.torque.commands == [[30.0] * 6]
    assert adapter.read_angles(100) == [25.0] * 6
    assert adapter.read_optional("temperature", 100) == [25.0] * 6


@pytest.mark.parametrize("values", ([1] * 5, [1] * 7, [-1] * 6, [101] * 6))
def test_invalid_commands_are_rejected(hand_and_adapter, values):
    _, adapter = hand_and_adapter
    with pytest.raises(ValueError):
        adapter.set_angles(values)


def test_close_is_idempotent(hand_and_adapter):
    hand, adapter = hand_and_adapter
    adapter.close()
    adapter.close()
    assert hand.closed


@pytest.mark.parametrize(
    ("model", "expected"),
    (
        ("l6", {"side": "right", "interface_name": "can2", "interface_type": "socketcan"}),
        ("o20", {"side": "right", "interface_type": "socketcan", "socketcan_channel": "can2"}),
        ("o30i", {"interface_type": "socketcan", "socketcan_channel": "can2"}),
        ("l30", {"interface_type": "socketcan", "channel": "can2"}),
    ),
)
def test_sdk_constructor_mapping(model, expected):
    calls = []

    class Factory:
        def __init__(self, **kwargs):
            calls.append(kwargs)

    sdk = SimpleNamespace(
        L6=Factory,
        O20=Factory,
        O30i=Factory,
        L30=Factory,
    )
    create_sdk_hand(
        model,
        side="right",
        interface_name="can2",
        interface_type="socketcan",
        sdk_module=sdk,
    )
    assert calls == [expected]


def test_canfd_commands_use_model_specific_integer_ranges():
    hand = FakeHand()
    adapter = HandAdapter("l30", hand_factory=lambda *args, **kwargs: hand)
    adapter.set_speeds([250] * 17)
    adapter.set_torques([800] * 17)
    assert hand.speed.commands == [[250] * 17]
    assert hand.torque.commands == [[800] * 17]
    with pytest.raises(ValueError, match="integers"):
        adapter.set_speeds([10.5] * 17)


def test_sdk_flag_values_are_converted_for_ros_messages():
    class FaultCode(Flag):
        NONE = 0
        OVER_CURRENT = 1

    assert _as_float_list([FaultCode.NONE, FaultCode.OVER_CURRENT]) == [0.0, 1.0]
