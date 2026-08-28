"""Typed ROS command subscriptions backed by non-blocking SDK setters."""

# ROS callbacks convert all SDK failures into diagnostics instead of escaping.
# ruff: noqa: BLE001

from __future__ import annotations

import threading
import time
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from linkerbot_hand import (
    CommandHandle,
    CommandSemantic,
    DeviceCapabilities,
    HandDevice,
)
from rclpy.lifecycle import LifecycleNode

from linkerbot_hand_msgs.msg import (
    HandCommandStatus,
    HandJointCommand,
    HandRawCommand,
)

from .command_router import (
    RoutedCommand,
    route_joint_command,
    route_raw_command,
)
from .conversion import command_status_to_message
from .qos import COMMAND_QOS, STATUS_QOS

_RAW_COMMANDS = (
    CommandSemantic.SPEEDS,
    CommandSemantic.ACCELERATIONS,
    CommandSemantic.CURRENT_LIMITS,
    CommandSemantic.EFFORT_LIMITS,
    CommandSemantic.TORQUES,
    CommandSemantic.MOTION_TIMES,
)


@dataclass(slots=True)
class _TrackedCommand:
    user_sequence: int
    handle: CommandHandle
    published_transition_count: int = 0


class CommandInterface:
    """Own command entities and bounded command-stage observation."""

    def __init__(
        self,
        node: LifecycleNode,
        *,
        device: Callable[[], HandDevice],
        is_active: Callable[[], bool],
        on_accepted: Callable[[], None],
        on_error: Callable[[BaseException], None],
        tracking_capacity: int = 512,
    ) -> None:
        self._node = node
        self._device = device
        self._is_active = is_active
        self._on_accepted = on_accepted
        self._on_error = on_error
        self._tracking_capacity = tracking_capacity
        self._status_publisher = node.create_publisher(
            HandCommandStatus,
            "~/command/status",
            STATUS_QOS,
        )
        self._subscriptions: list[Any] = []
        self._tracked: OrderedDict[int, _TrackedCommand] = OrderedDict()
        self._lock = threading.RLock()

    def activate(self, capabilities: DeviceCapabilities) -> None:
        """Create subscriptions for command families supported by the device."""

        if self._subscriptions:
            return
        subscriptions: list[Any] = []
        try:
            if capabilities.command(CommandSemantic.ANGLES) is not None:
                subscriptions.append(
                    self._node.create_subscription(
                        HandJointCommand,
                        "~/command/angles",
                        self._handle_angles,
                        COMMAND_QOS,
                    )
                )
            for semantic in _RAW_COMMANDS:
                if capabilities.command(semantic) is None:
                    continue
                subscriptions.append(
                    self._node.create_subscription(
                        HandRawCommand,
                        f"~/command/{semantic.value}",
                        lambda message, value=semantic: self._handle_raw(
                            value, message
                        ),
                        COMMAND_QOS,
                    )
                )
        except Exception:
            for subscription in subscriptions:
                self._node.destroy_subscription(subscription)
            raise
        self._subscriptions = subscriptions

    def deactivate(self) -> None:
        """Remove command subscriptions so inactive nodes cannot accept targets."""

        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions.clear()

    def track(self, routed: RoutedCommand) -> None:
        """Track and immediately publish the SDK stage visible at acceptance."""

        with self._lock:
            handle = routed.handle
            tracked = _TrackedCommand(routed.user_sequence, handle)
            self._make_room()
            self._tracked[handle.command_id] = tracked
            try:
                self._publish(tracked)
            except Exception as error:
                self._report_status_error(error)

    def publish_updates(self) -> int:
        """Publish changed handle histories without waiting on any handle."""

        with self._lock:
            publications = 0
            completed: list[int] = []
            for command_id, tracked in tuple(self._tracked.items()):
                transition_count = len(tracked.handle.history)
                if transition_count != tracked.published_transition_count:
                    self._publish(tracked, transition_count=transition_count)
                    publications += 1
                if tracked.handle.done and (
                    len(tracked.handle.history) == tracked.published_transition_count
                ):
                    completed.append(command_id)
            for command_id in completed:
                self._tracked.pop(command_id, None)
            return publications

    def destroy(self) -> None:
        """Destroy subscriptions and the command status publisher."""

        self.deactivate()
        with self._lock:
            self._tracked.clear()
        self._node.destroy_publisher(self._status_publisher)

    @property
    def tracked_count(self) -> int:
        with self._lock:
            return len(self._tracked)

    def _handle_angles(self, message: HandJointCommand) -> None:
        self._route(lambda: route_joint_command(self._device(), message))

    def _handle_raw(
        self,
        semantic: CommandSemantic,
        message: HandRawCommand,
    ) -> None:
        self._route(lambda: route_raw_command(self._device(), semantic, message))

    def _route(self, submit: Callable[[], RoutedCommand]) -> None:
        if not self._is_active():
            self._on_error(RuntimeError("lifecycle node is not active"))
            return
        try:
            routed = submit()
            self.track(routed)
            self._on_accepted()
        except Exception as error:
            self._on_error(error)

    def _publish(
        self,
        tracked: _TrackedCommand,
        *,
        transition_count: int | None = None,
    ) -> None:
        count = (
            len(tracked.handle.history)
            if transition_count is None
            else transition_count
        )
        now = self._node.get_clock().now().nanoseconds
        self._status_publisher.publish(
            command_status_to_message(
                tracked.handle,
                user_sequence=tracked.user_sequence,
                monotonic_now=time.monotonic(),
                ros_now_ns=now,
            )
        )
        tracked.published_transition_count = count

    def _make_room(self) -> None:
        if len(self._tracked) < self._tracking_capacity:
            return
        for command_id, tracked in tuple(self._tracked.items()):
            if tracked.handle.done:
                self._tracked.pop(command_id)
                return
        command_id, _tracked = self._tracked.popitem(last=False)
        self._report_status_error(
            RuntimeError(
                f"command status tracker capacity exceeded; dropped {command_id}"
            )
        )

    def _report_status_error(self, error: BaseException) -> None:
        self._node.get_logger().error(
            f"command status observation failed: {error}",
            throttle_duration_sec=5.0,
        )


__all__ = ["CommandInterface"]
