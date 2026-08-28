"""Persistent typed ROS client mirroring the HandDevice 1.0 command names."""

from __future__ import annotations

import math
import threading
import time
from collections import OrderedDict
from collections.abc import Sequence
from dataclasses import dataclass

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from linkerbot_hand_msgs.msg import (
    CommandCapability,
    HandCapabilities,
    HandCommandStatus,
    HandJointCommand,
    HandRawCommand,
)

from .qos import COMMAND_QOS, METADATA_QOS, STATUS_QOS

_RAW_COMMANDS = (
    "speeds",
    "accelerations",
    "current_limits",
    "effort_limits",
    "torques",
    "motion_times",
)
_TERMINAL_STAGES = {
    HandCommandStatus.STAGE_ACKNOWLEDGED,
    HandCommandStatus.STAGE_FAILED,
    HandCommandStatus.STAGE_SUPERSEDED,
}


@dataclass(frozen=True, slots=True)
class CommandPublication:
    """Result of publishing a target; SDK acceptance arrives on status topic."""

    user_sequence: int
    semantic: str
    representation: str
    values: tuple[float, ...]


class HandCommandClient:
    """Reuse typed publishers and capability-derived validation for one driver."""

    def __init__(
        self,
        node: Node,
        driver_name: str = "/linkerbot_hand",
        *,
        status_capacity: int = 512,
    ) -> None:
        root = driver_name.rstrip("/")
        if not root:
            raise ValueError("driver_name must not be empty")
        if type(status_capacity) is not int or status_capacity <= 0:
            raise ValueError("status_capacity must be a positive int")
        self._node = node
        self._driver_name = root
        self._status_capacity = status_capacity
        self._condition = threading.Condition(threading.RLock())
        self._capabilities: HandCapabilities | None = None
        self._statuses: OrderedDict[int, HandCommandStatus] = OrderedDict()
        self._sequence = 0
        self._destroyed = False
        self._angle_publisher: Publisher | None = None
        self._raw_publishers: dict[str, Publisher] = {}
        self._subscriptions: list[Subscription] = []
        try:
            self._angle_publisher = node.create_publisher(
                HandJointCommand,
                self._topic("command/angles"),
                COMMAND_QOS,
            )
            for semantic in _RAW_COMMANDS:
                self._raw_publishers[semantic] = node.create_publisher(
                    HandRawCommand,
                    self._topic(f"command/{semantic}"),
                    COMMAND_QOS,
                )
            self._subscriptions.extend(
                (
                    node.create_subscription(
                        HandCapabilities,
                        self._topic("capabilities"),
                        self._update_capabilities,
                        METADATA_QOS,
                    ),
                    node.create_subscription(
                        HandCommandStatus,
                        self._topic("command/status"),
                        self._update_status,
                        STATUS_QOS,
                    ),
                )
            )
        except Exception:
            self._destroy_ros_entities()
            self._destroyed = True
            raise

    @property
    def driver_name(self) -> str:
        return self._driver_name

    @property
    def model(self) -> str | None:
        with self._condition:
            return (
                None
                if self._capabilities is None
                else self._capabilities.identity.model
            )

    @property
    def joint_names(self) -> tuple[str, ...]:
        with self._condition:
            if self._capabilities is None:
                return ()
            return tuple(joint.name for joint in self._capabilities.joints)

    def is_ready(self, semantic: str = "angles") -> bool:
        """Return whether capability metadata and one driver subscription exist."""

        with self._condition:
            self._require_alive()
            if self._capabilities is None or not self._capabilities.identity.verified:
                return False
            capability = self._command_capability(semantic)
            if capability is None:
                return False
            publisher = self._publisher_for(semantic)
            return publisher is not None and publisher.get_subscription_count() > 0

    def wait_until_ready(self, timeout_s: float, semantic: str = "angles") -> bool:
        """Wait while another executor thread processes discovery and metadata."""

        timeout = _positive_timeout(timeout_s)
        deadline = time.monotonic() + timeout
        while not self.is_ready(semantic):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            with self._condition:
                self._condition.wait(timeout=min(remaining, 0.05))
        return True

    def set_percent_angles(self, values: Sequence[float]) -> CommandPublication:
        """Publish strict ``0..100`` targets in capability joint order."""

        normalized = self._validate_percent_angles(values)
        sequence = self._next_sequence()
        self._require_angle_publisher().publish(
            HandJointCommand(
                user_sequence=sequence,
                representation=HandJointCommand.REPRESENTATION_PERCENT,
                values=normalized,
            )
        )
        return CommandPublication(sequence, "angles", "percent", normalized)

    def set_angles(self, values: Sequence[float]) -> CommandPublication:
        """Publish exact protocol angle values in capability joint order."""

        normalized = self._validate_raw("angles", values)
        sequence = self._next_sequence()
        self._require_angle_publisher().publish(
            HandJointCommand(
                user_sequence=sequence,
                representation=HandJointCommand.REPRESENTATION_RAW,
                values=normalized,
            )
        )
        return CommandPublication(sequence, "angles", "raw", normalized)

    def set_speeds(self, values: Sequence[float]) -> CommandPublication:
        return self._set_raw("speeds", values)

    def set_accelerations(self, values: Sequence[float]) -> CommandPublication:
        return self._set_raw("accelerations", values)

    def set_current_limits(self, values: Sequence[float]) -> CommandPublication:
        return self._set_raw("current_limits", values)

    def set_effort_limits(self, values: Sequence[float]) -> CommandPublication:
        return self._set_raw("effort_limits", values)

    def set_torques(self, values: Sequence[float]) -> CommandPublication:
        return self._set_raw("torques", values)

    def set_motion_times(self, values: Sequence[float]) -> CommandPublication:
        return self._set_raw("motion_times", values)

    def get_status(self, user_sequence: int) -> HandCommandStatus | None:
        """Return the latest cached SDK stage for one published command."""

        with self._condition:
            self._require_alive()
            return self._statuses.get(user_sequence)

    def wait_for_terminal_status(
        self,
        user_sequence: int,
        timeout_s: float,
    ) -> HandCommandStatus | None:
        """Wait for protocol completion, never for mechanical target arrival."""

        timeout = _positive_timeout(timeout_s)
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                self._require_alive()
                status = self._statuses.get(user_sequence)
                if status is not None and (
                    status.stage in _TERMINAL_STAGES
                    or (
                        status.stage == HandCommandStatus.STAGE_DISPATCHED
                        and status.ack_support == HandCommandStatus.ACK_UNSUPPORTED
                    )
                ):
                    return status
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(timeout=remaining)

    def destroy(self) -> None:
        with self._condition:
            if self._destroyed:
                return
            self._destroyed = True
            self._destroy_ros_entities()
            self._condition.notify_all()

    def _set_raw(
        self,
        semantic: str,
        values: Sequence[float],
    ) -> CommandPublication:
        normalized = self._validate_raw(semantic, values)
        sequence = self._next_sequence()
        self._raw_publishers[semantic].publish(
            HandRawCommand(user_sequence=sequence, raw=normalized)
        )
        return CommandPublication(sequence, semantic, "raw", normalized)

    def _validate_percent_angles(
        self,
        values: Sequence[float],
    ) -> tuple[float, ...]:
        normalized = _finite_values(values)
        capability = self._require_command_capability("angles")
        if len(normalized) != len(capability.channel_names):
            raise ValueError("angles length does not match capability joint order")
        if any(value < 0 or value > 100 for value in normalized):
            raise ValueError("percent angles must be within 0..100")
        if any(not mapping.supported for mapping in capability.percent_mappings):
            raise ValueError("one or more angle axes lack a percent mapping")
        return normalized

    def _validate_raw(
        self,
        semantic: str,
        values: Sequence[float],
    ) -> tuple[float, ...]:
        normalized = _finite_values(values)
        capability = self._require_command_capability(semantic)
        if len(normalized) != len(capability.channel_names):
            raise ValueError(f"{semantic} length does not match capability channels")
        for value, raw_range in zip(normalized, capability.raw_ranges, strict=True):
            if raw_range.integer_only and not value.is_integer():
                raise ValueError(f"{semantic} requires integer protocol values")
            if not raw_range.minimum <= value <= raw_range.maximum:
                raise ValueError(f"{semantic} value is outside the declared range")
        return normalized

    def _next_sequence(self) -> int:
        with self._condition:
            self._require_alive()
            self._sequence += 1
            return self._sequence

    def _update_capabilities(self, message: HandCapabilities) -> None:
        with self._condition:
            self._capabilities = message
            self._condition.notify_all()

    def _update_status(self, message: HandCommandStatus) -> None:
        with self._condition:
            if message.user_sequence == 0:
                return
            self._statuses[message.user_sequence] = message
            self._statuses.move_to_end(message.user_sequence)
            while len(self._statuses) > self._status_capacity:
                self._statuses.popitem(last=False)
            self._condition.notify_all()

    def _require_command_capability(self, semantic: str) -> CommandCapability:
        with self._condition:
            self._require_alive()
            if self._capabilities is None or not self._capabilities.identity.verified:
                raise RuntimeError("driver capability is not identity-verified")
            capability = self._command_capability(semantic)
            if capability is None:
                raise RuntimeError(f"driver does not declare {semantic} support")
            publisher = self._publisher_for(semantic)
            if publisher is None or publisher.get_subscription_count() <= 0:
                raise RuntimeError(f"driver {semantic} command interface is not active")
            return capability

    def _command_capability(self, semantic: str) -> CommandCapability | None:
        if self._capabilities is None:
            return None
        return next(
            (
                capability
                for capability in self._capabilities.commands
                if capability.semantic == semantic
            ),
            None,
        )

    def _require_alive(self) -> None:
        if self._destroyed:
            raise RuntimeError("hand command client has been destroyed")

    def _require_angle_publisher(self) -> Publisher:
        self._require_alive()
        if self._angle_publisher is None:
            raise RuntimeError("angle publisher is unavailable")
        return self._angle_publisher

    def _publisher_for(self, semantic: str) -> Publisher | None:
        if semantic == "angles":
            return self._angle_publisher
        return self._raw_publishers.get(semantic)

    def _topic(self, suffix: str) -> str:
        return f"{self._driver_name}/{suffix}"

    def _destroy_ros_entities(self) -> None:
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        for publisher in self._raw_publishers.values():
            self._node.destroy_publisher(publisher)
        if self._angle_publisher is not None:
            self._node.destroy_publisher(self._angle_publisher)
            self._angle_publisher = None
        self._subscriptions.clear()
        self._raw_publishers.clear()


def _finite_values(values: Sequence[float]) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise TypeError("values must be a numeric sequence")
    materialized = tuple(values)
    if any(isinstance(value, bool) for value in materialized):
        raise TypeError("values must not contain bool")
    try:
        normalized = tuple(float(value) for value in materialized)
    except (TypeError, ValueError) as error:
        raise TypeError("values must be a numeric sequence") from error
    if any(not math.isfinite(value) for value in normalized):
        raise ValueError("values must be finite")
    return normalized


def _positive_timeout(value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError("timeout_s must be finite and positive")
    return float(value)


__all__ = ["CommandPublication", "HandCommandClient"]
