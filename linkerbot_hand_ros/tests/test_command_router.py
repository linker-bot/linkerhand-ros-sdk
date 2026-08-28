from dataclasses import dataclass, field

import pytest
from builtin_interfaces.msg import Time
from linkerbot_hand import CommandSemantic, ValidationError
from linkerbot_hand_ros.command_router import (
    DiscreteOperation,
    route_discrete_operation,
    route_joint_command,
    route_raw_command,
)

from linkerbot_hand_msgs.msg import HandJointCommand, HandRawCommand


class WaitTrap:
    def wait(self, *_args, **_kwargs):
        raise AssertionError("a ROS command callback must never wait on a handle")


@dataclass(frozen=True)
class IntegerRange:
    integer_only: bool = True


@dataclass(frozen=True)
class IntegerCommand:
    channels: tuple[str, ...] = ("first", "second")

    @staticmethod
    def range_for(_index):
        return IntegerRange()


class RecordingCapabilities:
    @staticmethod
    def command(_semantic):
        return IntegerCommand()


@dataclass
class RecordingDevice:
    calls: list[tuple[str, tuple[int | float, ...]]] = field(default_factory=list)
    handle: object = field(default_factory=WaitTrap)
    error: Exception | None = None

    @staticmethod
    def get_capabilities():
        return RecordingCapabilities()

    def __getattr__(self, name):
        def submit(values=()):
            normalized = tuple(values)
            self.calls.append((name, normalized))
            if self.error is not None:
                raise self.error
            return self.handle

        return submit


@pytest.mark.parametrize(
    "representation, expected_method",
    [
        (HandJointCommand.REPRESENTATION_PERCENT, "set_percent_angles"),
        (HandJointCommand.REPRESENTATION_RAW, "set_angles"),
    ],
)
def test_joint_router_uses_one_explicit_angle_representation(
    representation,
    expected_method,
):
    device = RecordingDevice()
    message = HandJointCommand(
        client_id=23,
        user_sequence=41,
        publisher_stamp=Time(sec=12, nanosec=34),
        representation=representation,
        values=[1.0, 2.0],
    )

    routed = route_joint_command(device, message)

    assert routed.client_id == 23
    assert routed.user_sequence == 41
    assert routed.publisher_stamp == Time(sec=12, nanosec=34)
    assert routed.handle is device.handle
    expected_values = (1.0, 2.0) if representation == 1 else (1, 2)
    assert device.calls == [(expected_method, expected_values)]


def test_joint_router_rejects_unknown_representation_before_sdk_submission():
    device = RecordingDevice()

    with pytest.raises(ValidationError, match="representation"):
        route_joint_command(
            device,
            HandJointCommand(
                client_id=1,
                user_sequence=1,
                representation=99,
                values=[1.0],
            ),
        )

    assert not device.calls


def test_joint_router_rejects_zero_client_id_before_sdk_submission():
    device = RecordingDevice()

    with pytest.raises(ValidationError, match="client_id must be a non-zero uint64"):
        route_joint_command(
            device,
            HandJointCommand(
                user_sequence=1,
                representation=HandJointCommand.REPRESENTATION_PERCENT,
                values=[1.0],
            ),
        )

    assert not device.calls


@pytest.mark.parametrize(
    "semantic, expected_method",
    [
        (CommandSemantic.SPEEDS, "set_speeds"),
        (CommandSemantic.ACCELERATIONS, "set_accelerations"),
        (CommandSemantic.CURRENT_LIMITS, "set_current_limits"),
        (CommandSemantic.EFFORT_LIMITS, "set_effort_limits"),
        (CommandSemantic.TORQUES, "set_torques"),
        (CommandSemantic.MOTION_TIMES, "set_motion_times"),
        (
            CommandSemantic.STALL_DETECTION_TIMES,
            "set_stall_detection_times",
        ),
        (CommandSemantic.STALL_THRESHOLDS, "set_stall_thresholds"),
        (CommandSemantic.STALL_HOLD_CURRENTS, "set_stall_hold_currents"),
    ],
)
def test_raw_router_recovers_integer_protocol_values_from_ros_float64(
    semantic,
    expected_method,
):
    device = RecordingDevice()
    message = HandRawCommand(
        client_id=24,
        user_sequence=7,
        publisher_stamp=Time(sec=13, nanosec=35),
        raw=[0.0, 255.0],
    )

    routed = route_raw_command(device, semantic, message)

    assert routed.client_id == 24
    assert routed.user_sequence == 7
    assert routed.publisher_stamp == Time(sec=13, nanosec=35)
    assert routed.handle is device.handle
    assert device.calls == [(expected_method, (0, 255))]


def test_value_validation_is_delegated_to_the_sdk_setter():
    expected = ValidationError("SDK rejected the device-specific range")
    device = RecordingDevice(error=expected)

    with pytest.raises(ValidationError) as raised:
        route_raw_command(
            device,
            CommandSemantic.CURRENT_LIMITS,
            HandRawCommand(client_id=1, user_sequence=1, raw=[9999.0]),
        )

    assert raised.value is expected
    assert device.calls == [("set_current_limits", (9999.0,))]


def test_raw_router_rejects_zero_client_id_before_sdk_submission():
    device = RecordingDevice()

    with pytest.raises(ValidationError, match="client_id must be a non-zero uint64"):
        route_raw_command(
            device,
            CommandSemantic.SPEEDS,
            HandRawCommand(user_sequence=1, raw=[1.0]),
        )

    assert not device.calls


@pytest.mark.parametrize("operation", list(DiscreteOperation))
def test_discrete_router_submits_bounded_sdk_operation_without_waiting(operation):
    device = RecordingDevice()

    routed = route_discrete_operation(device, operation, user_sequence=5)

    assert routed.user_sequence == 5
    assert routed.client_id == 0
    assert routed.publisher_stamp == Time()
    assert routed.handle is device.handle
    assert device.calls == [(operation.value, ())]


def test_discrete_router_rejects_user_sequence_outside_uint64():
    with pytest.raises(ValidationError, match="fit uint64"):
        route_discrete_operation(
            RecordingDevice(),
            DiscreteOperation.CLEAR_FAULTS,
            user_sequence=1 << 64,
        )


def test_discrete_router_rejects_client_id_outside_uint64():
    with pytest.raises(ValidationError, match="client_id must fit uint64"):
        route_discrete_operation(
            RecordingDevice(),
            DiscreteOperation.CLEAR_FAULTS,
            client_id=1 << 64,
        )
