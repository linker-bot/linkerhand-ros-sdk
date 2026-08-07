"""ROS 2 node exposing the Linkerbot hand adapter."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from .adapter import HandAdapter


class LinkerbotHandNode(Node):
    def __init__(self, adapter_factory: Callable[..., HandAdapter] = HandAdapter) -> None:
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

        poll_rate = float(self.get_parameter("poll_rate").value)
        self._timeout_ms = float(self.get_parameter("timeout_ms").value)
        if poll_rate <= 0.0:
            raise ValueError("poll_rate must be positive")
        if self._timeout_ms <= 0.0:
            raise ValueError("timeout_ms must be positive")

        self._adapter = adapter_factory(
            str(self.get_parameter("model").value),
            side=str(self.get_parameter("side").value),
            interface_name=str(self.get_parameter("interface_name").value),
            interface_type=str(self.get_parameter("interface_type").value),
            library_path=str(self.get_parameter("library_path").value) or None,
        )
        self._hand_lock = threading.Lock()
        self._last_error_log = 0.0
        self._publish_optional = bool(
            self.get_parameter("publish_optional_state").value
        )
        optional_state_rate = float(self.get_parameter("optional_state_rate").value)
        if self._publish_optional and optional_state_rate <= 0.0:
            raise ValueError("optional_state_rate must be positive")

        self._angle_publisher = self.create_publisher(
            Float64MultiArray, "~/state/angles", 10
        )
        self._joint_names_publisher = self.create_publisher(
            String, "~/joint_names", 1
        )
        self._optional_publishers = {
            name: self.create_publisher(Float64MultiArray, f"~/state/{name}", 10)
            for name in ("temperature", "current", "fault")
        }
        self.create_subscription(
            Float64MultiArray,
            "~/command/angles",
            lambda message: self._run_command("angles", message.data),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "~/command/speeds",
            lambda message: self._run_command("speeds", message.data),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "~/command/torques",
            lambda message: self._run_command("torques", message.data),
            10,
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
        self.create_timer(1.0, self._publish_joint_names)
        self.get_logger().info(
            f"connected model={self._adapter.model} "
            f"joints={len(self._adapter.joint_names)}"
        )

    def _publish_joint_names(self) -> None:
        message = String()
        message.data = ",".join(self._adapter.joint_names)
        self._joint_names_publisher.publish(message)

    @staticmethod
    def _message(values: list[float]) -> Float64MultiArray:
        message = Float64MultiArray()
        message.data = values
        return message

    def _run_command(self, command: str, values: Any) -> None:
        try:
            with self._hand_lock:
                getattr(self._adapter, f"set_{command}")(values)
        except Exception as error:  # ROS callbacks must keep the executor alive.
            self.get_logger().error(f"rejected {command} command: {error}")

    def _poll(self) -> None:
        try:
            with self._hand_lock:
                angles = self._adapter.read_angles(self._timeout_ms)
            self._angle_publisher.publish(self._message(angles))
        except Exception as error:
            now = time.monotonic()
            if now - self._last_error_log >= 5.0:
                self.get_logger().error(f"hand state read failed: {error}")
                self._last_error_log = now

    def _poll_optional(self) -> None:
        for name, publisher in self._optional_publishers.items():
            try:
                with self._hand_lock:
                    values = self._adapter.read_optional(name, self._timeout_ms)
                if values is not None:
                    publisher.publish(self._message(values))
            except Exception as error:
                self.get_logger().debug(f"{name} state read failed: {error}")

    def _read_now(self, _request: Trigger.Request, response: Trigger.Response):
        try:
            with self._hand_lock:
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
            with self._hand_lock:
                self._adapter.clear_faults()
            response.success = True
            response.message = "fault clear command sent"
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def destroy_node(self) -> bool:
        self._adapter.close()
        return super().destroy_node()


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
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
