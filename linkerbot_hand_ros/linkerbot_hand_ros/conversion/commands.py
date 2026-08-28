"""Convert SDK command-handle delivery history into ROS status messages."""

from __future__ import annotations

from builtin_interfaces.msg import Time
from linkerbot_hand import (
    AckSupport,
    CommandHandle,
    CommandStage,
    ValidationError,
)

from linkerbot_hand_msgs.msg import HandCommandStatus, HandCommandTransition

from .time import monotonic_timestamp_to_message

_STAGE_VALUES = {
    CommandStage.ACCEPTED: HandCommandStatus.STAGE_ACCEPTED,
    CommandStage.DISPATCHED: HandCommandStatus.STAGE_DISPATCHED,
    CommandStage.ACKNOWLEDGED: HandCommandStatus.STAGE_ACKNOWLEDGED,
    CommandStage.FAILED: HandCommandStatus.STAGE_FAILED,
    CommandStage.SUPERSEDED: HandCommandStatus.STAGE_SUPERSEDED,
}
_ACK_VALUES = {
    AckSupport.UNSUPPORTED: HandCommandStatus.ACK_UNSUPPORTED,
    AckSupport.SUPPORTED: HandCommandStatus.ACK_SUPPORTED,
}
_UINT64_MAX = (1 << 64) - 1


def command_status_to_message(
    handle: CommandHandle,
    *,
    client_id: int,
    user_sequence: int,
    publisher_stamp: Time,
    driver_received_at: float,
    monotonic_now: float,
    ros_now_ns: int,
) -> HandCommandStatus:
    """Snapshot one handle without waiting for a later command stage."""

    if not isinstance(handle, CommandHandle):
        raise ValidationError("handle must be CommandHandle")
    for value, name in ((client_id, "client_id"), (user_sequence, "user_sequence")):
        if type(value) is not int or not 0 <= value <= _UINT64_MAX:
            raise ValidationError(f"{name} must fit uint64")
    if not isinstance(publisher_stamp, Time):
        raise ValidationError("publisher_stamp must be builtin_interfaces/Time")
    if handle.command_id > _UINT64_MAX:
        raise ValidationError("command_id must fit uint64")
    history = handle.history
    latest = history[-1]
    error = handle.error if latest.stage is CommandStage.FAILED else None
    return HandCommandStatus(
        client_id=client_id,
        user_sequence=user_sequence,
        publisher_stamp=publisher_stamp,
        driver_received_stamp=monotonic_timestamp_to_message(
            driver_received_at,
            monotonic_now=monotonic_now,
            ros_now_ns=ros_now_ns,
        ),
        command_id=handle.command_id,
        family=handle.family.value,
        stage=_STAGE_VALUES[latest.stage],
        ack_support=_ACK_VALUES[handle.ack_support],
        sent_frames=latest.sent_frames,
        error_code="" if error is None else error.code.value,
        error_message="" if error is None else error.message,
        transitions=[
            HandCommandTransition(
                stage=_STAGE_VALUES[transition.stage],
                stamp=monotonic_timestamp_to_message(
                    transition.timestamp,
                    monotonic_now=monotonic_now,
                    ros_now_ns=ros_now_ns,
                ),
                sent_frames=transition.sent_frames,
            )
            for transition in history
        ],
    )


__all__ = ["command_status_to_message"]
