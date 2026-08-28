"""ROS entity ownership and zero-I/O publication for one configured driver."""

# ROS callback boundaries report arbitrary SDK/backend exceptions.
# ruff: noqa: BLE001

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from diagnostic_msgs.msg import DiagnosticArray
from linkerbot_hand import DeviceCapabilities, HandDevice, LifecycleError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode

from .command_interface import CommandInterface
from .diagnostics import build_diagnostic_status
from .driver_config import DriverConfiguration
from .management import ManagementInterface
from .metrics import DriverMetrics
from .qos import STATE_QOS
from .state_publisher import SnapshotPublisher


class DriverInterfaces:
    """Own every ROS entity created during the configured lifecycle state."""

    def __init__(
        self,
        node: LifecycleNode,
        configuration: DriverConfiguration,
        *,
        device: Callable[[], HandDevice],
        is_active: Callable[[], bool],
        error_text: Callable[[BaseException], str],
    ) -> None:
        self._node = node
        self._configuration = configuration
        self._device = device
        self._is_active = is_active
        self._error_text = error_text
        self._metrics = DriverMetrics()
        self._state_callback_group = MutuallyExclusiveCallbackGroup()
        self._capability_callback_group = MutuallyExclusiveCallbackGroup()
        self._status_callback_group = MutuallyExclusiveCallbackGroup()
        self._diagnostic_callback_group = MutuallyExclusiveCallbackGroup()
        self.snapshot: SnapshotPublisher | None = None
        self.commands: CommandInterface | None = None
        self.management: ManagementInterface | None = None
        self.diagnostic_publisher: Any | None = None
        self.state_timer: Any | None = None
        self.capability_timer: Any | None = None
        self.status_timer: Any | None = None
        self.diagnostic_timer: Any | None = None
        try:
            self._create()
        except Exception:
            try:
                self.destroy()
            except Exception as cleanup_error:
                self._node.get_logger().error(
                    "partial ROS interface cleanup failed: "
                    f"{self._error_text(cleanup_error)}"
                )
            raise

    def activate(self, capabilities: DeviceCapabilities) -> None:
        """Enable admin work, command subscriptions, and cached publication."""

        self._require_management().activate()
        self._require_commands().activate(capabilities)
        self._require_snapshot().reset()
        self._require_snapshot().publish_capabilities(capabilities)
        self.apply_rates(self._configuration)
        self.publish_cached_state()

    def deactivate(self) -> None:
        """Disable active entities and stop the bounded admin worker."""

        self.cancel_active_timers()
        errors: list[BaseException] = []
        if self.commands is not None:
            try:
                self.commands.deactivate()
            except Exception as error:
                errors.append(error)
        if self.management is not None:
            try:
                self.management.deactivate()
            except Exception as error:
                errors.append(error)
        if errors:
            raise LifecycleError(
                "one or more active ROS interfaces did not stop cleanly",
                operation="deactivate_ros_interfaces",
                cause=errors[0],
            )

    def apply_rates(self, configuration: DriverConfiguration) -> None:
        """Apply validated ROS timer rates and retain the new snapshot."""

        self._configuration = configuration
        self._set_timer_rate(
            self.state_timer,
            configuration.publish_rate_hz,
            enabled=self._is_active(),
        )
        self._set_timer_rate(
            self.diagnostic_timer,
            configuration.diagnostic_rate_hz,
            enabled=True,
        )
        self._set_timer_rate(
            self.capability_timer,
            10.0,
            enabled=self._is_active(),
        )
        self._set_timer_rate(self.status_timer, 100.0, enabled=self._is_active())

    def publish_capabilities(self, capabilities: DeviceCapabilities) -> None:
        self._require_snapshot().publish_capabilities(capabilities)

    def publish_cached_state(self) -> None:
        """Read only immutable SDK memory and publish changed sample signatures."""

        if not self._is_active():
            return
        try:
            device = self._device()
            capabilities = device.get_capabilities()
            publications = self._require_snapshot().publish_snapshot(
                device.get_snapshot(),
                capabilities,
                ros_now_ns=self._node.get_clock().now().nanoseconds,
            )
            if publications:
                self._metrics.record_state_success()
        except Exception as error:
            self._metrics.record_state_error(error)
            self._node.get_logger().error(
                f"cached state publication failed: {self._error_text(error)}",
                throttle_duration_sec=5.0,
            )

    def publish_runtime_capabilities(self) -> None:
        """Republish zero-I/O SDK capability changes such as firmware fallback."""

        if not self._is_active():
            return
        try:
            self._require_snapshot().publish_capabilities_if_changed(
                self._device().get_capabilities()
            )
        except Exception as error:
            self._node.get_logger().error(
                f"capability publication failed: {self._error_text(error)}",
                throttle_duration_sec=5.0,
            )

    def publish_command_status(self) -> None:
        if not self._is_active() or self.commands is None:
            return
        try:
            self.commands.publish_updates()
        except Exception as error:
            self._node.get_logger().error(
                f"command status publication failed: {self._error_text(error)}",
                throttle_duration_sec=5.0,
            )

    def publish_diagnostics(self) -> None:
        """Publish diagnostics from ``get_health()`` and no other SDK state call."""

        if self.diagnostic_publisher is None:
            return
        try:
            status = build_diagnostic_status(
                node_name=self._node.get_fully_qualified_name(),
                model=self._configuration.model,
                active=self._is_active(),
                health=self._device().get_health(),
            )
            message = DiagnosticArray()
            message.header.stamp = self._node.get_clock().now().to_msg()
            message.status = [status]
            self.diagnostic_publisher.publish(message)
        except Exception as error:
            self._node.get_logger().error(
                f"diagnostic publication failed: {self._error_text(error)}",
                throttle_duration_sec=5.0,
            )

    def cancel_active_timers(self) -> None:
        for timer in (self.state_timer, self.capability_timer, self.status_timer):
            if timer is not None:
                timer.cancel()

    def destroy(self) -> None:
        """Destroy all entities, attempting every owner even after one failure."""

        errors: list[BaseException] = []
        for timer_name in (
            "state_timer",
            "capability_timer",
            "status_timer",
            "diagnostic_timer",
        ):
            timer = getattr(self, timer_name)
            if timer is not None:
                self._node.destroy_timer(timer)
                setattr(self, timer_name, None)
        for attribute in ("management", "commands", "snapshot"):
            interface = getattr(self, attribute)
            if interface is None:
                continue
            try:
                interface.destroy()
            except Exception as error:
                errors.append(error)
            else:
                setattr(self, attribute, None)
        if self.diagnostic_publisher is not None:
            self._node.destroy_publisher(self.diagnostic_publisher)
            self.diagnostic_publisher = None
        if errors:
            raise LifecycleError(
                "one or more ROS interfaces did not shut down cleanly",
                operation="destroy_ros_interfaces",
                cause=errors[0],
            )

    def _create(self) -> None:
        self.snapshot = SnapshotPublisher(self._node)
        self.commands = CommandInterface(
            self._node,
            device=self._device,
            is_active=self._is_active,
            on_accepted=self._metrics.record_command_success,
            on_error=self._record_command_error,
        )
        self.management = ManagementInterface(
            self._node,
            device=self._device,
            is_active=self._is_active,
            track=self._require_commands().track,
            shutdown_timeout_s=self._configuration.device_config.shutdown_timeout_s,
            worker_capacity=(self._configuration.device_config.discrete_queue_capacity),
        )
        self.diagnostic_publisher = self._node.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            STATE_QOS,
        )
        self.state_timer = self._node.create_timer(
            self._timer_period(self._configuration.publish_rate_hz),
            self.publish_cached_state,
            callback_group=self._state_callback_group,
        )
        self.state_timer.cancel()
        self.capability_timer = self._node.create_timer(
            0.1,
            self.publish_runtime_capabilities,
            callback_group=self._capability_callback_group,
        )
        self.capability_timer.cancel()
        self.status_timer = self._node.create_timer(
            0.01,
            self.publish_command_status,
            callback_group=self._status_callback_group,
        )
        self.status_timer.cancel()
        self.diagnostic_timer = self._node.create_timer(
            self._timer_period(self._configuration.diagnostic_rate_hz),
            self.publish_diagnostics,
            callback_group=self._diagnostic_callback_group,
        )
        if self._configuration.diagnostic_rate_hz == 0:
            self.diagnostic_timer.cancel()

    def _record_command_error(self, error: BaseException) -> None:
        exception = error if isinstance(error, Exception) else RuntimeError(str(error))
        self._metrics.record_command_error(exception)
        self._node.get_logger().error(f"command rejected: {self._error_text(error)}")

    def _require_snapshot(self) -> SnapshotPublisher:
        if self.snapshot is None:
            raise LifecycleError("state interface is not configured")
        return self.snapshot

    def _require_commands(self) -> CommandInterface:
        if self.commands is None:
            raise LifecycleError("command interface is not configured")
        return self.commands

    def _require_management(self) -> ManagementInterface:
        if self.management is None:
            raise LifecycleError("management interface is not configured")
        return self.management

    @staticmethod
    def _timer_period(rate_hz: float) -> float:
        return 1.0 if rate_hz == 0 else 1.0 / rate_hz

    @staticmethod
    def _set_timer_rate(timer: Any | None, rate_hz: float, *, enabled: bool) -> None:
        if timer is None:
            return
        if rate_hz == 0 or not enabled:
            timer.cancel()
            return
        timer.timer_period_ns = int(1_000_000_000 / rate_hz)
        timer.reset()


__all__ = ["DriverInterfaces"]
