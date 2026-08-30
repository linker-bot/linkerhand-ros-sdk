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

from builtin_interfaces.msg import Time
from linkerbot_hand import (
    CommandHandle,
    CommandSemantic,
    DeviceCapabilities,
    HandDevice,
)
from rclpy.lifecycle import LifecycleNode

from linkerbot_hand_msgs.msg import (
    HandCommandStatus,
    HandCommandStatusBatch,
    HandJointCommand,
    HandRawCommand,
)

from .command_router import (
    RoutedCommand,
    route_joint_command,
    route_raw_command,
)
from .conversion import command_status_to_message
from .qos import (
    COMMAND_QOS,
    HIGH_RATE_PUBLISHER_EVENTS,
    HIGH_RATE_SUBSCRIPTION_EVENTS,
    STATUS_QOS,
)

_RAW_COMMANDS = (
    CommandSemantic.SPEEDS,
    CommandSemantic.ACCELERATIONS,
    CommandSemantic.CURRENT_LIMITS,
    CommandSemantic.EFFORT_LIMITS,
    CommandSemantic.TORQUES,
    CommandSemantic.MOTION_TIMES,
)
_STATUS_BATCH_CAPACITY = 512


@dataclass(slots=True)
class _TrackedCommand:
    client_id: int
    user_sequence: int
    publisher_stamp: Time
    driver_received_at: float
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
        if (
            type(tracking_capacity) is not int
            or not 0 < tracking_capacity <= _STATUS_BATCH_CAPACITY
        ):
            raise ValueError(
                f"tracking_capacity must be within 1..{_STATUS_BATCH_CAPACITY}"
            )
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
            HandCommandStatusBatch,
            "~/command/status",
            STATUS_QOS,
            event_callbacks=HIGH_RATE_PUBLISHER_EVENTS,
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
                        event_callbacks=HIGH_RATE_SUBSCRIPTION_EVENTS,
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
                        event_callbacks=HIGH_RATE_SUBSCRIPTION_EVENTS,
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

    def track(
        self,
        routed: RoutedCommand,
        driver_received_at: float | None = None,
    ) -> None:
        """Register an accepted handle without publishing from the command callback."""

        with self._lock:
            if not self._enabled:
                return
            handle = routed.handle
            tracked = _TrackedCommand(
                client_id=routed.client_id,
                user_sequence=routed.user_sequence,
                publisher_stamp=routed.publisher_stamp,
                driver_received_at=(
                    time.monotonic()
                    if driver_received_at is None
                    else driver_received_at
                ),
                handle=handle,
            )
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
        """Publish changed handle histories in one bounded status batch."""

        with self._lock:
            if not self._enabled:
                return 0
            candidates = tuple(self._tracked.items())

        changed: list[tuple[int, _TrackedCommand]] = []
        for command_id, tracked in candidates:
            transition_count = len(tracked.handle.history)
            if transition_count == tracked.published_transition_count:
                self._remove_if_complete(command_id, tracked)
                continue
            changed.append((command_id, tracked))
        if not changed:
            return 0

        ros_now_ns = self._node.get_clock().now().nanoseconds
        monotonic_now = time.monotonic()
        converted: list[tuple[int, _TrackedCommand, HandCommandStatus, int]] = []
        for command_id, tracked in changed:
            try:
                message = self._message(
                    tracked,
                    ros_now_ns=ros_now_ns,
                    monotonic_now=monotonic_now,
                )
            except Exception as error:
                self._report_status_error(error)
                continue
            converted.append((command_id, tracked, message, len(message.transitions)))
        if not converted:
            return 0

        try:
            with self._publication_lock:
                with self._lock:
                    if not self._enabled:
                        return 0
                    active = tuple(
                        item
                        for item in converted
                        if self._tracked.get(item[0]) is item[1]
                    )
                if not active:
                    return 0
                self._status_publisher.publish(
                    HandCommandStatusBatch(statuses=[item[2] for item in active])
                )
        except Exception as error:
            self._report_status_error(error)
            return 0

        for command_id, tracked, _message, published_count in active:
            with self._lock:
                if self._tracked.get(command_id) is not tracked:
                    continue
                tracked.published_transition_count = max(
                    tracked.published_transition_count,
                    published_count,
                )
            self._remove_if_complete(command_id, tracked)
        return len(active)

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
        driver_received_at = time.monotonic()
        if not self._is_active():
            self._on_error(RuntimeError("lifecycle node is not active"))
            return
        try:
            routed = submit()
            self.track(routed, driver_received_at)
            self._on_accepted()
        except Exception as error:
            self._on_error(error)

    def _message(
        self,
        tracked: _TrackedCommand,
        *,
        ros_now_ns: int,
        monotonic_now: float,
    ) -> HandCommandStatus:
        return command_status_to_message(
            tracked.handle,
            client_id=tracked.client_id,
            user_sequence=tracked.user_sequence,
            publisher_stamp=tracked.publisher_stamp,
            driver_received_at=tracked.driver_received_at,
            monotonic_now=monotonic_now,
            ros_now_ns=ros_now_ns,
        )

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
