"""ROS 2 node exposing the Linkerbot hand adapter."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from .adapter import HandAdapter
from .metrics import DriverMetrics
from .qos import COMMAND_QOS, METADATA_QOS, STATE_QOS


class _PriorityIoLock:
    """Serialize SDK I/O while giving on-demand operations priority over polls."""

    def __init__(self) -> None:
        self._io_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._priority_waiters = 0

    @contextmanager
    def priority(self) -> Iterator[None]:
        with self._state_lock:
            self._priority_waiters += 1
        try:
            with self._io_lock:
                yield
        finally:
            with self._state_lock:
                self._priority_waiters -= 1

    @contextmanager
    def poll(self) -> Iterator[bool]:
        with self._state_lock:
            acquired = self._priority_waiters == 0 and self._io_lock.acquire(
                blocking=False
            )
        try:
            yield acquired
        finally:
            if acquired:
                self._io_lock.release()


class LinkerbotHandNode(Node):
    def __init__(
        self, adapter_factory: Callable[..., HandAdapter] = HandAdapter
    ) -> None:
        super().__init__("linkerbot_hand")
        self.declare_parameter("model", "l6")
        self.declare_parameter("side", "left")
        self.declare_parameter("interface_name", "can0")
        self.declare_parameter("interface_type", "socketcan")
        self.declare_parameter("library_path", "")
        self.declare_parameter("poll_rate", 30.0)
        self.declare_parameter("timeout_ms", 100.0)
        self.declare_parameter("publish_optional_state", True)
        self.declare_parameter("optional_state_rate", 1.0)
        self.declare_parameter("diagnostic_rate", 1.0)

        poll_rate = float(self.get_parameter("poll_rate").value)
        self._timeout_ms = float(self.get_parameter("timeout_ms").value)
        diagnostic_rate = float(self.get_parameter("diagnostic_rate").value)
        if poll_rate <= 0.0:
            raise ValueError("poll_rate must be positive")
        if self._timeout_ms <= 0.0:
            raise ValueError("timeout_ms must be positive")
        if diagnostic_rate <= 0.0:
            raise ValueError("diagnostic_rate must be positive")

        self._adapter = adapter_factory(
            str(self.get_parameter("model").value),
            side=str(self.get_parameter("side").value),
            interface_name=str(self.get_parameter("interface_name").value),
            interface_type=str(self.get_parameter("interface_type").value),
            library_path=str(self.get_parameter("library_path").value) or None,
        )
        self._hand_io = _PriorityIoLock()
        self._metrics = DriverMetrics()
        self._state_stale_after_sec = max(
            1.0,
            (3.0 / poll_rate) + (self._timeout_ms / 1000.0),
        )
        self._last_error_log = 0.0
        self._publish_optional = bool(
            self.get_parameter("publish_optional_state").value
        )
        optional_state_rate = float(self.get_parameter("optional_state_rate").value)
        if self._publish_optional and optional_state_rate <= 0.0:
            raise ValueError("optional_state_rate must be positive")

        self._angle_publisher = self.create_publisher(
            Float64MultiArray, "~/state/angles", STATE_QOS
        )
        self._joint_names_publisher = self.create_publisher(
            String, "~/joint_names", METADATA_QOS
        )
        self._model_publisher = self.create_publisher(
            String, "~/model", METADATA_QOS
        )
        self._optional_publishers = {
            name: self.create_publisher(
                Float64MultiArray, f"~/state/{name}", STATE_QOS
            )
            for name in ("temperature", "current", "fault")
        }
        self._accepted_publishers = {
            command: self.create_publisher(
                Float64MultiArray,
                f"~/command/accepted/{command}",
                METADATA_QOS,
            )
            for command in ("angles", "speeds", "torques")
        }
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", STATE_QOS
        )
        self.create_subscription(
            Float64MultiArray,
            "~/command/angles",
            lambda message: self._run_command("angles", message),
            COMMAND_QOS,
        )
        self.create_subscription(
            Float64MultiArray,
            "~/command/speeds",
            lambda message: self._run_command("speeds", message),
            COMMAND_QOS,
        )
        self.create_subscription(
            Float64MultiArray,
            "~/command/torques",
            lambda message: self._run_command("torques", message),
            COMMAND_QOS,
        )

        io_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger, "~/read_now", self._read_now, callback_group=io_group
        )
        self.create_service(
            Trigger, "~/clear_faults", self._clear_faults, callback_group=io_group
        )
        self.create_timer(1.0 / poll_rate, self._poll, callback_group=io_group)
        if self._publish_optional:
            self.create_timer(
                1.0 / optional_state_rate,
                self._poll_optional,
                callback_group=io_group,
            )
        self.create_timer(1.0, self._publish_metadata)
        self.create_timer(1.0 / diagnostic_rate, self._publish_diagnostics)
        self._publish_metadata()
        self.get_logger().info(
            f"connected model={self._adapter.model} "
            f"joints={len(self._adapter.joint_names)}"
        )

    def _publish_metadata(self) -> None:
        joint_names = String()
        joint_names.data = ",".join(self._adapter.joint_names)
        self._joint_names_publisher.publish(joint_names)
        model = String()
        model.data = self._adapter.model
        self._model_publisher.publish(model)

    @staticmethod
    def _message(values: list[float]) -> Float64MultiArray:
        message = Float64MultiArray()
        message.data = values
        return message

    def _run_command(
        self, command: str, message: Float64MultiArray
    ) -> None:
        try:
            with self._hand_io.priority():
                accepted = getattr(self._adapter, f"set_{command}")(message.data)
            self._accepted_publishers[command].publish(self._message(accepted))
            self._metrics.record_command_success()
        except Exception as error:  # ROS callbacks must keep the executor alive.
            self._metrics.record_command_error(error)
            self.get_logger().error(f"rejected {command} command: {error}")

    def _poll(self) -> None:
        try:
            with self._hand_io.poll() as acquired:
                if not acquired:
                    return
                angles = self._adapter.read_angles(self._timeout_ms)
            self._angle_publisher.publish(self._message(angles))
            self._metrics.record_state_success()
        except Exception as error:
            self._metrics.record_state_error(error)
            now = time.monotonic()
            if now - self._last_error_log >= 5.0:
                self.get_logger().error(f"hand state read failed: {error}")
                self._last_error_log = now

    def _poll_optional(self) -> None:
        for name, publisher in self._optional_publishers.items():
            try:
                with self._hand_io.poll() as acquired:
                    if not acquired:
                        return
                    values = self._adapter.read_optional(name, self._timeout_ms)
                if values is not None:
                    publisher.publish(self._message(values))
            except Exception as error:
                self.get_logger().debug(f"{name} state read failed: {error}")

    @staticmethod
    def _diagnostic_value(key: str, value: object) -> KeyValue:
        return KeyValue(key=key, value=str(value))

    def _publish_diagnostics(self) -> None:
        snapshot = self._metrics.snapshot()
        status = DiagnosticStatus()
        status.name = f"{self.get_fully_qualified_name()}: hand_driver"
        status.hardware_id = self._adapter.model
        if snapshot.state_successes == 0 and snapshot.state_errors > 0:
            status.level = DiagnosticStatus.ERROR
            status.message = "state reads are failing before first sample"
        elif snapshot.state_successes == 0:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for first state sample"
        elif (
            snapshot.last_state_age_sec is not None
            and snapshot.last_state_age_sec > self._state_stale_after_sec
        ):
            status.level = DiagnosticStatus.ERROR
            status.message = "hand state is stale"
        elif snapshot.latest_state_failed or snapshot.latest_command_failed:
            status.level = DiagnosticStatus.WARN
            status.message = "latest driver operation failed"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "operational"
        status.values = [
            self._diagnostic_value("model", self._adapter.model),
            self._diagnostic_value("joint_count", len(self._adapter.joint_names)),
            self._diagnostic_value("uptime_sec", f"{snapshot.uptime_sec:.3f}"),
            self._diagnostic_value("state_successes", snapshot.state_successes),
            self._diagnostic_value("state_errors", snapshot.state_errors),
            self._diagnostic_value("command_successes", snapshot.command_successes),
            self._diagnostic_value("command_errors", snapshot.command_errors),
            self._diagnostic_value(
                "last_state_age_sec",
                "never"
                if snapshot.last_state_age_sec is None
                else f"{snapshot.last_state_age_sec:.3f}",
            ),
            self._diagnostic_value(
                "last_command_age_sec",
                "never"
                if snapshot.last_command_age_sec is None
                else f"{snapshot.last_command_age_sec:.3f}",
            ),
            self._diagnostic_value("last_error", snapshot.last_error),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostic_publisher.publish(message)

    def _read_now(self, _request: Trigger.Request, response: Trigger.Response):
        try:
            with self._hand_io.priority():
                angles = self._adapter.read_angles(self._timeout_ms)
            self._angle_publisher.publish(self._message(angles))
            response.success = True
            response.message = "angle state published"
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def _clear_faults(self, _request: Trigger.Request, response: Trigger.Response):
        try:
            with self._hand_io.priority():
                self._adapter.clear_faults()
            response.success = True
            response.message = "fault clear command sent"
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def destroy_node(self) -> bool:
        try:
            self._adapter.close()
        finally:
            destroyed = super().destroy_node()
        return destroyed


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
