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
        # Commands share the lifecycle control group so admission and deactivation
        # cannot pass each other while observation callbacks run independently.
        self._callback_group = node.default_callback_group
        self._status_publisher = node.create_publisher(
            HandCommandStatus,
            "~/command/status",
            STATUS_QOS,
        )
        self._subscriptions: list[Any] = []
        self._tracked: OrderedDict[int, _TrackedCommand] = OrderedDict()
        self._lock = threading.RLock()
        self._publication_lock = threading.Lock()
        self._enabled = False

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
                        callback_group=self._callback_group,
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
                        callback_group=self._callback_group,
                    )
                )
        except Exception:
            for subscription in subscriptions:
                self._node.destroy_subscription(subscription)
            raise
        self._subscriptions = subscriptions
        with self._lock:
            self._enabled = True

    def deactivate(self) -> None:
        """Remove command subscriptions so inactive nodes cannot accept targets."""

        with self._lock:
            self._enabled = False
            self._tracked.clear()
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions.clear()
        with self._publication_lock:
            pass

    def track(self, routed: RoutedCommand) -> None:
        """Register an accepted handle without publishing from the command callback."""

        with self._lock:
            if not self._enabled:
                return
            handle = routed.handle
            tracked = _TrackedCommand(routed.user_sequence, handle)
            dropped_command_id = self._make_room()
            self._tracked[handle.command_id] = tracked
        if dropped_command_id is not None:
            self._report_status_error(
                RuntimeError(
                    "command status tracker capacity exceeded; "
                    f"dropped {dropped_command_id}"
                )
            )

    def publish_updates(self) -> int:
        """Publish changed handle histories without waiting on any handle."""

        with self._lock:
            if not self._enabled:
                return 0
            candidates = tuple(self._tracked.items())

        publications = 0
        for command_id, tracked in candidates:
            transition_count = len(tracked.handle.history)
            if transition_count == tracked.published_transition_count:
                self._remove_if_complete(command_id, tracked)
                continue
            try:
                with self._publication_lock:
                    with self._lock:
                        if (
                            not self._enabled
                            or self._tracked.get(command_id) is not tracked
                        ):
                            continue
                    published_count = self._publish(tracked)
            except Exception as error:
                self._report_status_error(error)
                continue
            publications += 1
            with self._lock:
                if self._tracked.get(command_id) is not tracked:
                    continue
                tracked.published_transition_count = max(
                    tracked.published_transition_count,
                    published_count,
                )
            self._remove_if_complete(command_id, tracked)
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
    ) -> int:
        now = self._node.get_clock().now().nanoseconds
        message = command_status_to_message(
            tracked.handle,
            user_sequence=tracked.user_sequence,
            monotonic_now=time.monotonic(),
            ros_now_ns=now,
        )
        self._status_publisher.publish(message)
        return len(message.transitions)

    def _make_room(self) -> int | None:
        if len(self._tracked) < self._tracking_capacity:
            return None
        for command_id, tracked in tuple(self._tracked.items()):
            if tracked.handle.done:
                self._tracked.pop(command_id)
                return command_id
        command_id, _tracked = self._tracked.popitem(last=False)
        return command_id

    def _remove_if_complete(
        self,
        command_id: int,
        tracked: _TrackedCommand,
    ) -> None:
        if not tracked.handle.done:
            return
        transition_count = len(tracked.handle.history)
        with self._lock:
            if (
                self._tracked.get(command_id) is tracked
                and transition_count == tracked.published_transition_count
            ):
                self._tracked.pop(command_id)

    def _report_status_error(self, error: BaseException) -> None:
        self._node.get_logger().error(
            f"command status observation failed: {error}",
            throttle_duration_sec=5.0,
        )


__all__ = ["CommandInterface"]
