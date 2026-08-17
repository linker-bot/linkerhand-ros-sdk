"""Lifecycle ROS 2 driver for Linkerbot hands."""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from linkerbot_hand import (
    CommandHandle,
    HandSDKError,
    SensorSample,
    SensorType,
)
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from .adapter import CONTINUOUS_COMMAND_NAMES, HandAdapter
from .configuration import (
    DRIVER_PARAMETERS,
    RUNTIME_RATE_PARAMETERS,
    STATIC_PARAMETERS,
    DriverConfiguration,
    read_driver_configuration,
)
from .diagnostics import build_diagnostic_status
from .metrics import DriverMetrics
from .qos import COMMAND_QOS, METADATA_QOS, STATE_QOS

_STATE_SENSORS = {
    "angles": SensorType.ANGLES,
    "speeds": SensorType.SPEEDS,
    "torques": SensorType.TORQUES,
    "temperature": SensorType.TEMPERATURE,
    "current": SensorType.CURRENT,
    "fault": SensorType.FAULTS,
    "voltage": SensorType.VOLTAGE,
    "acceleration": SensorType.ACCELERATION,
    "motion_time": SensorType.MOTION_TIME,
}


class LinkerbotHandNode(LifecycleNode):
    """One lifecycle-managed ROS owner for one SDK ``HandDevice``."""

    def __init__(
        self,
        adapter_factory: Callable[..., HandAdapter] = HandAdapter,
        **kwargs: Any,
    ) -> None:
        super().__init__("linkerbot_hand", **kwargs)
        self.declare_parameter("model", "l6")
        self.declare_parameter("side", "left")
        self.declare_parameter("transport_options_json", "{}")
        # Deprecated compatibility parameters used only when the JSON object is empty.
        self.declare_parameter("interface_name", "can0")
        self.declare_parameter("interface_type", "socketcan")
        self.declare_parameter("library_path", "")
        self.declare_parameter("command_rate_hz", 100.0)
        self.declare_parameter("auto_reconnect", False)
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("angles_hz", 60.0)
        self.declare_parameter("speeds_hz", 0.0)
        self.declare_parameter("torques_hz", 0.0)
        self.declare_parameter("force_hz", 0.0)
        self.declare_parameter("temperature_hz", 1.0)
        self.declare_parameter("current_hz", 10.0)
        self.declare_parameter("faults_hz", 5.0)
        self.declare_parameter("voltage_hz", 0.0)
        self.declare_parameter("acceleration_hz", 0.0)
        self.declare_parameter("motion_time_hz", 0.0)
        self.declare_parameter("diagnostic_rate", 1.0)

        self._adapter_factory = adapter_factory
        self._adapter: HandAdapter | None = None
        self._configuration: DriverConfiguration | None = None
        self._active = False
        self._destroying = False
        self._driver_destroyed = False
        self._metrics = DriverMetrics()
        self._latest_handles: dict[str, CommandHandle] = {}

        self._state_publishers: dict[str, Any] = {}
        self._accepted_publishers: dict[str, Any] = {}
        self._joint_names_publisher: Any | None = None
        self._model_publisher: Any | None = None
        self._diagnostic_publisher: Any | None = None
        self._command_subscriptions: list[Any] = []
        self._driver_services: list[Any] = []
        self._state_timer: Any | None = None
        self._metadata_timer: Any | None = None
        self._diagnostic_timer: Any | None = None
        self._parameter_callback_handle = self.add_on_set_parameters_callback(
            self._on_set_parameters
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            configuration = read_driver_configuration(
                lambda name: self.get_parameter(name).value
            )
            self._configuration = configuration
            self._adapter = self._adapter_factory(
                configuration.model,
                side=configuration.side,
                transport_options=configuration.transport_options,
                session_config=configuration.session_config,
            )
            self._create_ros_entities(configuration)
            result = super().on_configure(state)
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError("managed ROS entity configuration failed")
            self._publish_metadata()
            capabilities = self._require_adapter().capabilities
            self.get_logger().info(
                f"configured model={capabilities.model} "
                f"joints={capabilities.joint_count}; device remains disconnected"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as error:
            self.get_logger().error(f"configure failed: {self._error_text(error)}")
            self._active = False
            self._release_adapter(close=True)
            self._adapter = None
            self._configuration = None
            self._destroy_ros_entities()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        adapter = self._adapter
        configuration = self._configuration
        if adapter is None or configuration is None:
            self.get_logger().error("activate failed: hand is not configured")
            return TransitionCallbackReturn.FAILURE
        try:
            adapter.open()
            adapter.start_polling(configuration.polling_plan(adapter.capabilities))
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError("managed ROS entity activation failed")
            self._active = True
            self._create_command_subscriptions()
            if self._state_timer is not None:
                if configuration.publish_rate_hz > 0.0:
                    self._state_timer.reset()
                else:
                    self._state_timer.cancel()
            self._publish_metadata()
            capabilities = adapter.capabilities
            self.get_logger().info(
                f"activated model={capabilities.model} "
                f"side={capabilities.side.value} firmware={capabilities.firmware or 'unknown'}"
            )
            return TransitionCallbackReturn.SUCCESS
        except Exception as error:
            self.get_logger().error(f"activate failed: {self._error_text(error)}")
            self._active = False
            self._destroy_command_subscriptions()
            if self._state_timer is not None:
                self._state_timer.cancel()
            try:
                super().on_deactivate(state)
            except Exception as rollback_error:
                self.get_logger().error(
                    f"activate rollback failed: {self._error_text(rollback_error)}"
                )
            self._release_adapter(close=True)
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._active = False
        self._destroy_command_subscriptions()
        if self._state_timer is not None:
            self._state_timer.cancel()
        adapter = self._adapter
        try:
            if adapter is not None:
                adapter.stop_polling()
            result = super().on_deactivate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                return result
        except Exception as error:
            self.get_logger().error(f"deactivate failed: {self._error_text(error)}")
            return TransitionCallbackReturn.ERROR
        self.get_logger().info("deactivated; SDK polling stopped")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._active = False
        self._destroy_command_subscriptions()
        try:
            managed_result = super().on_cleanup(state)
        except Exception as error:
            managed_result = TransitionCallbackReturn.ERROR
            self.get_logger().error(
                f"managed ROS cleanup failed: {self._error_text(error)}"
            )
        released = self._release_adapter(close=True)
        if released:
            self._adapter = None
            self._configuration = None
        self._latest_handles.clear()
        self._destroy_ros_entities()
        if managed_result != TransitionCallbackReturn.SUCCESS or not released:
            return TransitionCallbackReturn.ERROR
        self.get_logger().info("cleaned up; SDK connection closed")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._active = False
        self._destroy_command_subscriptions()
        try:
            managed_result = super().on_shutdown(state)
        except Exception as error:
            managed_result = TransitionCallbackReturn.ERROR
            self.get_logger().error(
                f"managed ROS shutdown failed: {self._error_text(error)}"
            )
        released = self._release_adapter(close=True)
        if released:
            self._adapter = None
            self._configuration = None
        self._latest_handles.clear()
        self._destroy_ros_entities()
        if managed_result != TransitionCallbackReturn.SUCCESS or not released:
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._active = False
        self._destroy_command_subscriptions()
        if self._state_timer is not None:
            self._state_timer.cancel()
        try:
            managed_result = super().on_error(state)
        except Exception as error:
            managed_result = TransitionCallbackReturn.ERROR
            self.get_logger().error(
                f"managed ROS error cleanup failed: {self._error_text(error)}"
            )
        released = self._release_adapter(close=True)
        if released:
            self._adapter = None
            self._configuration = None
        self._latest_handles.clear()
        self._destroy_ros_entities()
        if managed_result != TransitionCallbackReturn.SUCCESS or not released:
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def _create_ros_entities(self, configuration: DriverConfiguration) -> None:
        for name in _STATE_SENSORS:
            self._state_publishers[name] = self.create_lifecycle_publisher(
                Float64MultiArray, f"~/state/{name}", STATE_QOS
            )
        self._state_publishers["raw_angles"] = self.create_lifecycle_publisher(
            Float64MultiArray, "~/state/raw_angles", STATE_QOS
        )
        for command in CONTINUOUS_COMMAND_NAMES:
            self._accepted_publishers[command] = self.create_publisher(
                Float64MultiArray,
                f"~/command/accepted/{command}",
                METADATA_QOS,
            )
        self._joint_names_publisher = self.create_publisher(
            String, "~/joint_names", METADATA_QOS
        )
        self._model_publisher = self.create_publisher(String, "~/model", METADATA_QOS)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", STATE_QOS
        )

        self._driver_services.append(
            self.create_service(
                Trigger,
                "~/read_now",
                self._read_now,
                callback_group=self.default_callback_group,
            )
        )
        for service_name, callback in (
            ("enable", self._enable),
            ("disable", self._disable),
            ("calibrate", self._calibrate),
        ):
            self._driver_services.append(
                self.create_service(
                    Trigger,
                    f"~/{service_name}",
                    callback,
                    callback_group=self.default_callback_group,
                )
            )
        self._driver_services.append(
            self.create_service(
                Trigger,
                "~/clear_faults",
                self._clear_faults,
                callback_group=self.default_callback_group,
            )
        )
        self._state_timer = self.create_timer(
            self._timer_period(configuration.publish_rate_hz),
            self._publish_cached_state,
        )
        self._state_timer.cancel()
        self._metadata_timer = self.create_timer(1.0, self._publish_metadata)
        self._diagnostic_timer = self.create_timer(
            self._timer_period(configuration.diagnostic_rate_hz),
            self._publish_diagnostics,
        )
        if configuration.diagnostic_rate_hz == 0.0:
            self._diagnostic_timer.cancel()

    @staticmethod
    def _timer_period(rate_hz: float) -> float:
        return 1.0 if rate_hz == 0.0 else 1.0 / rate_hz

    @staticmethod
    def _set_timer_rate(timer: Any | None, rate_hz: float, *, enabled: bool) -> None:
        if timer is None:
            return
        if rate_hz == 0.0 or not enabled:
            timer.cancel()
            return
        timer.timer_period_ns = int(1_000_000_000 / rate_hz)
        timer.reset()

    def _apply_timer_rates(self, configuration: DriverConfiguration) -> None:
        self._set_timer_rate(
            self._state_timer,
            configuration.publish_rate_hz,
            enabled=self._active,
        )
        self._set_timer_rate(
            self._diagnostic_timer,
            configuration.diagnostic_rate_hz,
            enabled=True,
        )

    def _on_set_parameters(
        self, parameters: list[Parameter]
    ) -> SetParametersResult:
        proposed = {
            parameter.name: parameter.value
            for parameter in parameters
            if parameter.name in DRIVER_PARAMETERS
        }
        static_updates = {
            parameter.name
            for parameter in parameters
            if parameter.name in STATIC_PARAMETERS
        }
        if self._configuration is not None and static_updates:
            names = ", ".join(sorted(static_updates))
            return SetParametersResult(
                successful=False,
                reason=(
                    f"static parameters require cleanup before update: {names}"
                ),
            )
        if self._configuration is None and proposed:
            try:
                read_driver_configuration(
                    lambda name: proposed.get(name, self.get_parameter(name).value)
                )
            except Exception as error:
                return SetParametersResult(
                    successful=False,
                    reason=self._error_text(error),
                )
            return SetParametersResult(successful=True)
        updates = {
            parameter.name: parameter.value
            for parameter in parameters
            if parameter.name in RUNTIME_RATE_PARAMETERS
        }
        if not updates:
            return SetParametersResult(successful=True)

        configuration = self._configuration
        try:
            updated = configuration.with_runtime_rates(updates)
            adapter = self._adapter
            old_plan = (
                None
                if adapter is None
                else configuration.polling_plan(adapter.capabilities)
            )
            if self._active and adapter is not None:
                adapter.start_polling(updated.polling_plan(adapter.capabilities))
            try:
                self._apply_timer_rates(updated)
            except Exception:
                if self._active and adapter is not None and old_plan is not None:
                    adapter.start_polling(old_plan)
                self._apply_timer_rates(configuration)
                raise
            self._configuration = updated
            return SetParametersResult(successful=True)
        except Exception as error:
            return SetParametersResult(
                successful=False,
                reason=self._error_text(error),
            )

    def _create_command_subscriptions(self) -> None:
        if self._command_subscriptions:
            return
        subscriptions: list[Any] = []
        try:
            for command in CONTINUOUS_COMMAND_NAMES:
                subscriptions.append(
                    self.create_subscription(
                        Float64MultiArray,
                        f"~/command/{command}",
                        lambda message, name=command: self._run_command(name, message),
                        COMMAND_QOS,
                    )
                )
        except Exception:
            for subscription in subscriptions:
                self.destroy_subscription(subscription)
            raise
        self._command_subscriptions = subscriptions

    def _destroy_command_subscriptions(self) -> None:
        for subscription in self._command_subscriptions:
            self.destroy_subscription(subscription)
        self._command_subscriptions.clear()

    def _destroy_ros_entities(self) -> None:
        self._destroy_command_subscriptions()
        for timer_name in (
            "_state_timer",
            "_metadata_timer",
            "_diagnostic_timer",
        ):
            timer = getattr(self, timer_name)
            if timer is not None:
                self.destroy_timer(timer)
                setattr(self, timer_name, None)
        for service in self._driver_services:
            self.destroy_service(service)
        self._driver_services.clear()
        for publisher in self._state_publishers.values():
            self.destroy_lifecycle_publisher(publisher)
        self._state_publishers.clear()
        for publisher in self._accepted_publishers.values():
            self.destroy_publisher(publisher)
        self._accepted_publishers.clear()
        for publisher_name in (
            "_joint_names_publisher",
            "_model_publisher",
            "_diagnostic_publisher",
        ):
            publisher = getattr(self, publisher_name)
            if publisher is not None:
                self.destroy_publisher(publisher)
                setattr(self, publisher_name, None)

    def _publish_metadata(self) -> None:
        adapter = self._adapter
        if adapter is None:
            return
        if self._joint_names_publisher is not None:
            self._joint_names_publisher.publish(
                String(data=",".join(adapter.joint_names))
            )
        if self._model_publisher is not None:
            self._model_publisher.publish(String(data=adapter.model))

    @staticmethod
    def _message(values: list[float]) -> Float64MultiArray:
        return Float64MultiArray(data=values)

    @staticmethod
    def _numeric_values(sample: SensorSample) -> list[float]:
        values: list[float] = []
        for value in sample.values:
            if value is None:
                values.append(math.nan)
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(
                    f"{sample.sensor.value} cache contains a non-numeric value"
                )
            numeric = float(value)
            if not math.isfinite(numeric):
                raise ValueError(
                    f"{sample.sensor.value} cache contains a non-finite value"
                )
            values.append(numeric)
        return values

    @staticmethod
    def _raw_numeric_values(sample: SensorSample) -> list[float] | None:
        if sample.raw_values is None:
            return None
        return [float(value) for value in sample.raw_values]

    def _run_command(self, command: str, message: Float64MultiArray) -> None:
        adapter = self._adapter
        if not self._active or adapter is None:
            self._metrics.record_command_error(RuntimeError("driver is not active"))
            self.get_logger().warning(
                f"rejected {command} command: lifecycle node is not active",
                throttle_duration_sec=5.0,
            )
            return
        try:
            handle = getattr(adapter, f"submit_{command}")(message.data)
        except Exception as error:
            self._metrics.record_command_error(error)
            self.get_logger().error(
                f"rejected {command} command: {self._error_text(error)}"
            )
            return

        self._latest_handles[command] = handle
        self._metrics.record_command_success()
        accepted = [float(value) for value in handle.command.values]
        try:
            self._accepted_publishers[command].publish(self._message(accepted))
            self._metrics.record_feedback_success()
        except Exception as error:
            self._metrics.record_feedback_error(error)
            self.get_logger().error(
                f"{command} command was accepted by SDK, but accepted feedback "
                f"publication failed: {self._error_text(error)}"
            )

    def _publish_cached_state(self) -> None:
        adapter = self._adapter
        configuration = self._configuration
        if not self._active or adapter is None or configuration is None:
            return
        try:
            snapshot = adapter.get_snapshot()
            enabled_sensors = {
                sensor for sensor, rate in configuration.sensor_rates if rate > 0.0
            }
            for topic_name, sensor in _STATE_SENSORS.items():
                if sensor not in enabled_sensors:
                    continue
                sample = snapshot.sample(sensor)
                if sample is None or not sample.valid or sample.stale:
                    continue
                publisher = self._state_publishers.get(topic_name)
                if publisher is not None:
                    publisher.publish(self._message(self._numeric_values(sample)))
                    if sensor is SensorType.ANGLES:
                        raw_values = self._raw_numeric_values(sample)
                        if raw_values is not None:
                            self._state_publishers["raw_angles"].publish(
                                self._message(raw_values)
                            )
                        self._metrics.record_state_success()
        except Exception as error:
            self._metrics.record_state_error(error)
            self.get_logger().error(
                f"cached state publication failed: {self._error_text(error)}",
                throttle_duration_sec=5.0,
            )

    def _publish_diagnostics(self) -> None:
        publisher = self._diagnostic_publisher
        adapter = self._adapter
        if publisher is None or adapter is None:
            return
        try:
            health = adapter.get_health()
            snapshot = adapter.get_snapshot()
            capabilities = adapter.capabilities
            status = build_diagnostic_status(
                node_name=self.get_fully_qualified_name(),
                active=self._active,
                capabilities=capabilities,
                health=health,
                snapshot=snapshot,
                metrics=self._metrics.snapshot(),
                latest_handles=self._latest_handles,
            )
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status = [status]
            publisher.publish(message)
        except Exception as error:
            self.get_logger().error(
                f"diagnostic publication failed: {self._error_text(error)}",
                throttle_duration_sec=5.0,
            )

    def _read_now(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        adapter = self._adapter
        if not self._active or adapter is None:
            response.success = False
            response.message = "lifecycle node is not active"
            return response
        try:
            sample = adapter.get_snapshot().angles
            if sample is None or not sample.valid or sample.stale:
                response.success = False
                response.message = "no fresh cached angle state is available"
                return response
            self._state_publishers["angles"].publish(
                self._message(self._numeric_values(sample))
            )
            raw_values = self._raw_numeric_values(sample)
            if raw_values is not None:
                self._state_publishers["raw_angles"].publish(
                    self._message(raw_values)
                )
            self._metrics.record_state_success()
            response.success = True
            response.message = "fresh cached angle state published (no device I/O)"
        except Exception as error:
            response.success = False
            response.message = self._error_text(error)
        return response

    def _clear_faults(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._run_discrete_operation(
            "clear_faults",
            "fault clearing",
            "clear_faults",
            response,
        )

    def _enable(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._run_discrete_operation(
            "enable", "device enable", "enable", response
        )

    def _disable(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._run_discrete_operation(
            "disable", "device disable", "enable", response
        )

    def _calibrate(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._run_discrete_operation(
            "calibrate", "device calibration", "calibration", response
        )

    def _run_discrete_operation(
        self,
        operation: str,
        description: str,
        feature: str,
        response: Trigger.Response,
    ) -> Trigger.Response:
        adapter = self._adapter
        if not self._active or adapter is None:
            error = RuntimeError("driver is not active")
            self._metrics.record_command_error(error)
            response.success = False
            response.message = "lifecycle node is not active"
            return response
        if not getattr(adapter.capabilities.features, feature):
            message = f"{description} is not supported by this model"
            error = RuntimeError(message)
            self._metrics.record_command_error(error)
            response.success = False
            response.message = message
            return response
        try:
            handle = getattr(adapter, operation)()
            self._latest_handles[operation] = handle
            self._metrics.record_command_success()
            response.success = True
            response.message = (
                f"{description} command accepted by SDK (id={handle.command_id}); "
                "device dispatch is asynchronous"
            )
        except Exception as error:
            self._metrics.record_command_error(error)
            response.success = False
            response.message = self._error_text(error)
        return response

    def _release_adapter(self, *, close: bool) -> bool:
        adapter = self._adapter
        if adapter is None:
            return True
        success = True
        try:
            adapter.stop_polling()
        except Exception as error:
            success = False
            self.get_logger().error(
                f"SDK stop_polling failed: {self._error_text(error)}"
            )
        if close:
            try:
                adapter.close()
            except Exception as error:
                success = False
                self.get_logger().error(f"SDK close failed: {self._error_text(error)}")
        return success

    def _require_adapter(self) -> HandAdapter:
        if self._adapter is None:
            raise RuntimeError("hand is not configured")
        return self._adapter

    @staticmethod
    def _error_text(error: BaseException) -> str:
        if isinstance(error, HandSDKError):
            return f"[{error.code.value}] {error}"
        return str(error)

    def destroy_node(self) -> None:
        if self._destroying or self._driver_destroyed:
            return
        self._destroying = True
        try:
            self._active = False
            released = self._release_adapter(close=True)
            if not released:
                self.get_logger().warning("retrying SDK shutdown once")
                released = self._release_adapter(close=True)
            if released:
                self._adapter = None
                self._configuration = None
            else:
                self.get_logger().error(
                    "SDK shutdown still failed after one bounded retry"
                )
            self._destroy_ros_entities()
            super().destroy_node()
            self._driver_destroyed = True
        finally:
            self._destroying = False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: LinkerbotHandNode | None = None
    executor = MultiThreadedExecutor(num_threads=2)
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
