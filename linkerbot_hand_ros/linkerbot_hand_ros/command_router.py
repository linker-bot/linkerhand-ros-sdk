"""Route typed ROS commands to the SDK's non-blocking ``set_*`` methods."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field
from enum import Enum

from builtin_interfaces.msg import Time
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
    SAVE_PARAMETERS = "save_parameters"


@dataclass(frozen=True, slots=True)
class RoutedCommand:
    """User correlation data paired with the accepted SDK handle."""

    client_id: int
    user_sequence: int
    publisher_stamp: Time = field(repr=False)
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
    _validate_client_id(message.client_id)
    if message.representation == HandJointCommand.REPRESENTATION_PERCENT:
        handle = device.set_percent_angles(message.values)
    elif message.representation == HandJointCommand.REPRESENTATION_RAW:
        handle = device.set_angles(
            _protocol_values(device, CommandSemantic.ANGLES, message.values)
        )
    else:
        raise ValidationError("joint command representation is unsupported")
    return RoutedCommand(
        client_id=message.client_id,
        user_sequence=message.user_sequence,
        publisher_stamp=message.publisher_stamp,
        handle=handle,
    )


def route_raw_command(
    device: HandDevice,
    semantic: CommandSemantic,
    message: HandRawCommand,
    *,
    require_client_id: bool = True,
) -> RoutedCommand:
    """Submit one explicitly selected protocol-raw command family."""

    if not isinstance(semantic, CommandSemantic):
        raise ValidationError("semantic must be CommandSemantic")
    if not isinstance(message, HandRawCommand):
        raise ValidationError("message must be HandRawCommand")
    if require_client_id:
        _validate_client_id(message.client_id)
    setter_name = _RAW_SETTERS.get(semantic)
    if setter_name is None:
        raise ValidationError(
            f"{semantic.value} requires a dedicated ROS command representation"
        )
    handle = getattr(device, setter_name)(
        _protocol_values(device, semantic, message.raw)
    )
    return RoutedCommand(
        client_id=message.client_id,
        user_sequence=message.user_sequence,
        publisher_stamp=message.publisher_stamp,
        handle=handle,
    )


def route_discrete_operation(
    device: HandDevice,
    operation: DiscreteOperation,
    *,
    client_id: int = 0,
    user_sequence: int = 0,
) -> RoutedCommand:
    """Submit one discrete SDK operation without waiting for dispatch or ACK."""

    if not isinstance(operation, DiscreteOperation):
        raise ValidationError("operation must be DiscreteOperation")
    for value, name in ((client_id, "client_id"), (user_sequence, "user_sequence")):
        if type(value) is not int or not 0 <= value <= _UINT64_MAX:
            raise ValidationError(f"{name} must fit uint64")
    handle = getattr(device, operation.value)()
    return RoutedCommand(
        client_id=client_id,
        user_sequence=user_sequence,
        publisher_stamp=Time(),
        handle=handle,
    )


def _protocol_values(
    device: HandDevice,
    semantic: CommandSemantic,
    values: Iterable[float],
) -> tuple[int | float, ...]:
    """Recover exact integer protocol values from ROS ``float64`` arrays."""

    command = device.get_capabilities().command(semantic)
    materialized = tuple(values)
    if command is None or len(materialized) != len(command.channels):
        return materialized
    normalized: list[int | float] = []
    for index, value in enumerate(materialized):
        raw_range = command.range_for(index)
        if raw_range.integer_only and isinstance(value, float) and value.is_integer():
            normalized.append(int(value))
        else:
            normalized.append(value)
    return tuple(normalized)


def _validate_client_id(value: object) -> None:
    if type(value) is not int or not 0 < value <= _UINT64_MAX:
        raise ValidationError("client_id must be a non-zero uint64")


__all__ = [
    "DiscreteOperation",
    "RoutedCommand",
    "route_discrete_operation",
    "route_joint_command",
    "route_raw_command",
]
