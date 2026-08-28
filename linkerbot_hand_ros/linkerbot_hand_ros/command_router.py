"""Route typed ROS commands to the SDK's non-blocking ``set_*`` methods."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from linkerbot_hand import (
    CommandHandle,
    CommandSemantic,
    HandDevice,
    ValidationError,
)

from linkerbot_hand_msgs.msg import HandJointCommand, HandRawCommand

_UINT64_MAX = (1 << 64) - 1


class DiscreteOperation(str, Enum):
    """Bounded FIFO operations exposed by the ROS management interface."""

    CLEAR_FAULTS = "clear_faults"
    ENABLE = "enable"
    DISABLE = "disable"
    CALIBRATE = "calibrate"


@dataclass(frozen=True, slots=True)
class RoutedCommand:
    """User correlation data paired with the accepted SDK handle."""

    user_sequence: int
    handle: CommandHandle


_RAW_SETTERS = {
    CommandSemantic.SPEEDS: "set_speeds",
    CommandSemantic.ACCELERATIONS: "set_accelerations",
    CommandSemantic.CURRENT_LIMITS: "set_current_limits",
    CommandSemantic.EFFORT_LIMITS: "set_effort_limits",
    CommandSemantic.TORQUES: "set_torques",
    CommandSemantic.MOTION_TIMES: "set_motion_times",
    CommandSemantic.STALL_DETECTION_TIMES: "set_stall_detection_times",
    CommandSemantic.STALL_THRESHOLDS: "set_stall_thresholds",
    CommandSemantic.STALL_HOLD_CURRENTS: "set_stall_hold_currents",
}


def route_joint_command(
    device: HandDevice,
    message: HandJointCommand,
) -> RoutedCommand:
    """Submit a percent or protocol-raw angle command and return immediately."""

    if not isinstance(message, HandJointCommand):
        raise ValidationError("message must be HandJointCommand")
    if message.representation == HandJointCommand.REPRESENTATION_PERCENT:
        handle = device.set_percent_angles(message.values)
    elif message.representation == HandJointCommand.REPRESENTATION_RAW:
        handle = device.set_angles(message.values)
    else:
        raise ValidationError("joint command representation is unsupported")
    return RoutedCommand(user_sequence=message.user_sequence, handle=handle)


def route_raw_command(
    device: HandDevice,
    semantic: CommandSemantic,
    message: HandRawCommand,
) -> RoutedCommand:
    """Submit one explicitly selected protocol-raw command family."""

    if not isinstance(semantic, CommandSemantic):
        raise ValidationError("semantic must be CommandSemantic")
    if not isinstance(message, HandRawCommand):
        raise ValidationError("message must be HandRawCommand")
    setter_name = _RAW_SETTERS.get(semantic)
    if setter_name is None:
        raise ValidationError(
            f"{semantic.value} requires a dedicated ROS command representation"
        )
    handle = getattr(device, setter_name)(message.raw)
    return RoutedCommand(user_sequence=message.user_sequence, handle=handle)


def route_discrete_operation(
    device: HandDevice,
    operation: DiscreteOperation,
    *,
    user_sequence: int = 0,
) -> RoutedCommand:
    """Submit one discrete SDK operation without waiting for dispatch or ACK."""

    if not isinstance(operation, DiscreteOperation):
        raise ValidationError("operation must be DiscreteOperation")
    if type(user_sequence) is not int or not 0 <= user_sequence <= _UINT64_MAX:
        raise ValidationError("user_sequence must fit uint64")
    handle = getattr(device, operation.value)()
    return RoutedCommand(user_sequence=user_sequence, handle=handle)


__all__ = [
    "DiscreteOperation",
    "RoutedCommand",
    "route_discrete_operation",
    "route_joint_command",
    "route_raw_command",
]
