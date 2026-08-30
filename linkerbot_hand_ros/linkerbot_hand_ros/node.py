"""Lifecycle ROS 2 driver for the single Hand SDK 1.0 facade."""

# Lifecycle callbacks must contain and report transport/backend failures.
# ruff: noqa: BLE001

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import rclpy
from linkerbot_hand import HandDevice, HandError
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.parameter import Parameter

from .driver_config import (
    DriverConfiguration,
    read_driver_configuration,
    validate_polling_plan,
)
from .driver_interfaces import DriverInterfaces
from .parameter_updates import declare_driver_parameters, prepare_parameter_update
from .transport_factory import create_hand_device

DeviceFactory = Callable[[DriverConfiguration], HandDevice]


class LinkerbotHandNode(LifecycleNode):
    """Map one ROS lifecycle to one SDK ``HandDevice`` owner."""

    def __init__(
        self,
        device_factory: DeviceFactory = create_hand_device,
        **kwargs: Any,
    ) -> None:
        super().__init__("linkerbot_hand", **kwargs)
        declare_driver_parameters(self)
        self._device_factory = device_factory
        self._configuration: DriverConfiguration | None = None
        self._device: HandDevice | None = None
        self._interfaces: DriverInterfaces | None = None
        self._active = False
        self._destroying = False
        self._driver_destroyed = False
        self._parameter_callback_handle = self.add_on_set_parameters_callback(
            self._on_set_parameters
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Construct configuration, transport, SDK facade, and ROS entities only."""

        try:
            configuration = read_driver_configuration(
                lambda name: self.get_parameter(name).value
            )
            self._configuration = configuration
            self._device = self._device_factory(configuration)
            self._interfaces = DriverInterfaces(
                self,
                configuration,
                device=self._require_device,
                is_active=lambda: self._active,
                error_text=self._error_text,
            )
            result = super().on_configure(state)
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError("managed ROS entity configuration failed")
            self._interfaces.publish_capabilities(
                self._require_device().get_capabilities()
            )
            self.get_logger().info(
                f"configured model={configuration.model}; device remains disconnected"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as error:
            self.get_logger().error(f"configure failed: {self._error_text(error)}")
            self._active = False
            self._close_device()
            self._destroy_interfaces()
            self._device = None
            self._configuration = None
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Open, verify, start SDK polling, then enable ROS control surfaces."""

        device = self._device
        configuration = self._configuration
        interfaces = self._interfaces
        if device is None or configuration is None or interfaces is None:
            self.get_logger().error("activate failed: hand is not configured")
            return TransitionCallbackReturn.FAILURE
        try:
            device.open()
            capabilities = device.get_capabilities()
            validate_polling_plan(capabilities, configuration.polling_plan)
            device.start_polling(configuration.polling_plan)
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError("managed ROS entity activation failed")
            self._active = True
            interfaces.activate(capabilities)
            identity = capabilities.identity
            self.get_logger().info(
                f"activated model={capabilities.model} side={identity.side.value} "
                f"firmware={identity.firmware or 'unknown'}"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as error:
            self.get_logger().error(f"activate failed: {self._error_text(error)}")
            self._rollback_activation(state)
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop accepting commands and SDK polling without closing the lease."""

        self._active = False
        success = self._deactivate_interfaces()
        success = self._stop_polling() and success
        try:
            result = super().on_deactivate(state)
            success = success and result == TransitionCallbackReturn.SUCCESS
        except Exception as error:
            success = False
            self.get_logger().error(
                f"managed ROS deactivate failed: {self._error_text(error)}"
            )
        if not success:
            return TransitionCallbackReturn.ERROR
        self.get_logger().info("deactivated; SDK polling stopped")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Close the SDK runtime and destroy all configured ROS entities."""

        success = self._release_runtime()
        try:
            result = super().on_cleanup(state)
            success = success and result == TransitionCallbackReturn.SUCCESS
        except Exception as error:
            success = False
            self.get_logger().error(
                f"managed ROS cleanup failed: {self._error_text(error)}"
            )
        success = self._destroy_interfaces() and success
        if success:
            self._device = None
            self._configuration = None
            self.get_logger().info("cleaned up; SDK connection closed")
            return TransitionCallbackReturn.SUCCESS
        return TransitionCallbackReturn.ERROR

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        return self._final_transition(super().on_shutdown, state, "shutdown")

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        return self._final_transition(super().on_error, state, "error cleanup")

    def _final_transition(
        self,
        managed_callback: Callable[[LifecycleState], TransitionCallbackReturn],
        state: LifecycleState,
        description: str,
    ) -> TransitionCallbackReturn:
        success = self._release_runtime()
        try:
            result = managed_callback(state)
            success = success and result == TransitionCallbackReturn.SUCCESS
        except Exception as error:
            success = False
            self.get_logger().error(
                f"managed ROS {description} failed: {self._error_text(error)}"
            )
        success = self._destroy_interfaces() and success
        if success:
            self._device = None
            self._configuration = None
            return TransitionCallbackReturn.SUCCESS
        return TransitionCallbackReturn.ERROR

    def _on_set_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        prepared = prepare_parameter_update(
            parameters,
            self._configuration,
            read_parameter=lambda name: self.get_parameter(name).value,
            error_text=self._error_text,
        )
        if not prepared.result.successful or prepared.configuration is None:
            return prepared.result
        previous = self._configuration
        if previous is None:
            return SetParametersResult(
                successful=False,
                reason="driver configuration is unavailable",
            )
        updated = prepared.configuration
        device = self._device
        interfaces = self._interfaces
        try:
            if self._active and device is not None:
                validate_polling_plan(device.get_capabilities(), updated.polling_plan)
                device.start_polling(updated.polling_plan)
            if interfaces is not None:
                interfaces.apply_rates(updated)
            self._configuration = updated
            return prepared.result
        except Exception as error:
            rollback_errors: list[BaseException] = []
            if self._active and device is not None:
                try:
                    device.start_polling(previous.polling_plan)
                except Exception as rollback_error:
                    rollback_errors.append(rollback_error)
            if interfaces is not None:
                try:
                    interfaces.apply_rates(previous)
                except Exception as rollback_error:
                    rollback_errors.append(rollback_error)
            reason = self._error_text(error)
            if rollback_errors:
                details = "; ".join(
                    self._error_text(rollback_error)
                    for rollback_error in rollback_errors
                )
                reason = f"{reason}; runtime rollback failed: {details}"
                self.get_logger().error(reason)
            return SetParametersResult(
                successful=False,
                reason=reason,
            )

    def _rollback_activation(self, state: LifecycleState) -> None:
        self._active = False
        self._deactivate_interfaces()
        self._stop_polling()
        try:
            super().on_deactivate(state)
        except Exception as error:
            self.get_logger().error(
                f"activate rollback failed: {self._error_text(error)}"
            )
        self._close_device()

    def _release_runtime(self) -> bool:
        self._active = False
        success = self._deactivate_interfaces()
        success = self._stop_polling() and success
        return self._close_device() and success

    def _deactivate_interfaces(self) -> bool:
        if self._interfaces is None:
            return True
        try:
            self._interfaces.deactivate()
            return True
        except Exception as error:
            self.get_logger().error(
                f"ROS interface deactivate failed: {self._error_text(error)}"
            )
            return False

    def _destroy_interfaces(self) -> bool:
        if self._interfaces is None:
            return True
        try:
            self._interfaces.destroy()
            self._interfaces = None
            return True
        except Exception as error:
            self.get_logger().error(
                f"ROS interface destruction failed: {self._error_text(error)}"
            )
            return False

    def _stop_polling(self) -> bool:
        if self._device is None:
            return True
        try:
            self._device.stop_polling()
            return True
        except Exception as error:
            self.get_logger().error(
                f"SDK stop_polling failed: {self._error_text(error)}"
            )
            return False

    def _close_device(self) -> bool:
        if self._device is None:
            return True
        try:
            self._device.close()
            return True
        except Exception as error:
            self.get_logger().error(f"SDK close failed: {self._error_text(error)}")
            return False

    def _require_device(self) -> HandDevice:
        if self._device is None:
            raise RuntimeError("hand is not configured")
        return self._device

    def _publish_cached_state(self) -> None:
        if self._interfaces is not None:
            self._interfaces.publish_cached_state()

    def _publish_command_status(self) -> None:
        if self._interfaces is not None:
            self._interfaces.publish_command_status()

    def _publish_diagnostics(self) -> None:
        if self._interfaces is not None:
            self._interfaces.publish_diagnostics()

    @staticmethod
    def _error_text(error: BaseException) -> str:
        if isinstance(error, HandError):
            return f"[{error.code.value}] {error}"
        return str(error)

    def destroy_node(self) -> None:
        if self._destroying or self._driver_destroyed:
            return
        self._destroying = True
        try:
            self._release_runtime()
            self._destroy_interfaces()
            self._device = None
            self._configuration = None
            super().destroy_node()
            self._driver_destroyed = True
        finally:
            self._destroying = False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: LinkerbotHandNode | None = None
    executor = SingleThreadedExecutor()
    try:
        node = LinkerbotHandNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
