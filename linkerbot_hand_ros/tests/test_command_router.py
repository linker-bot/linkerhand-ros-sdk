from dataclasses import dataclass, field

import pytest
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


@dataclass
class RecordingDevice:
    calls: list[tuple[str, tuple[float, ...]]] = field(default_factory=list)
    handle: object = field(default_factory=WaitTrap)
    error: Exception | None = None

    def __getattr__(self, name):
        def submit(values=()):
            normalized = tuple(float(value) for value in values)
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
        user_sequence=41,
        representation=representation,
        values=[1.0, 2.0],
    )

    routed = route_joint_command(device, message)

    assert routed.user_sequence == 41
    assert routed.handle is device.handle
    assert device.calls == [(expected_method, (1.0, 2.0))]


def test_joint_router_rejects_unknown_representation_before_sdk_submission():
    device = RecordingDevice()

    with pytest.raises(ValidationError, match="representation"):
        route_joint_command(
            device,
            HandJointCommand(user_sequence=1, representation=99, values=[1.0]),
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
def test_raw_router_maps_protocol_semantics_without_value_conversion(
    semantic,
    expected_method,
):
    device = RecordingDevice()
    message = HandRawCommand(user_sequence=7, raw=[0.0, 255.0])

    routed = route_raw_command(device, semantic, message)

    assert routed.user_sequence == 7
    assert routed.handle is device.handle
    assert device.calls == [(expected_method, (0.0, 255.0))]


def test_value_validation_is_delegated_to_the_sdk_setter():
    expected = ValidationError("SDK rejected the device-specific range")
    device = RecordingDevice(error=expected)

    with pytest.raises(ValidationError) as raised:
        route_raw_command(
            device,
            CommandSemantic.CURRENT_LIMITS,
            HandRawCommand(user_sequence=1, raw=[9999.0]),
        )

    assert raised.value is expected
    assert device.calls == [("set_current_limits", (9999.0,))]


@pytest.mark.parametrize("operation", list(DiscreteOperation))
def test_discrete_router_submits_bounded_sdk_operation_without_waiting(operation):
    device = RecordingDevice()

    routed = route_discrete_operation(device, operation, user_sequence=5)

    assert routed.user_sequence == 5
    assert routed.handle is device.handle
    assert device.calls == [(operation.value, ())]


def test_discrete_router_rejects_user_sequence_outside_uint64():
    with pytest.raises(ValidationError, match="fit uint64"):
        route_discrete_operation(
            RecordingDevice(),
            DiscreteOperation.CLEAR_FAULTS,
            user_sequence=1 << 64,
        )
