"""Print joint names and angle state from a running hand driver."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray, String


def _parse_args(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="监听灵巧手角度状态")
    parser.add_argument(
        "--names-topic",
        default="/linkerbot_hand/joint_names",
        help="关节名称话题",
    )
    parser.add_argument(
        "--angles-topic",
        default="/linkerbot_hand/state/angles",
        help="角度状态话题",
    )
    parser.add_argument(
        "--print-interval",
        type=float,
        default=0.5,
        help="角度输出最小间隔秒数",
    )
    return parser.parse_args(args)


class StateMonitor(Node):
    def __init__(
        self, names_topic: str, angles_topic: str, print_interval: float
    ) -> None:
        super().__init__("linkerbot_hand_state_monitor")
        self._joint_names: list[str] = []
        self._print_interval = print_interval
        self._last_print = 0.0
        self.create_subscription(String, names_topic, self._on_names, 10)
        self.create_subscription(Float64MultiArray, angles_topic, self._on_angles, 10)

    def _on_names(self, message: String) -> None:
        self._joint_names = [name for name in message.data.split(",") if name]
        self.get_logger().info("关节顺序: " + ", ".join(self._joint_names))

    def _on_angles(self, message: Float64MultiArray) -> None:
        now = time.monotonic()
        if now - self._last_print < self._print_interval:
            return
        self._last_print = now
        if not self._joint_names:
            self.get_logger().warning("尚未收到 joint_names，当前仅输出角度数组")
            self.get_logger().info(str(list(message.data)))
            return
        if len(message.data) != len(self._joint_names):
            self.get_logger().error(
                f"角度数量 {len(message.data)} 与关节数量 "
                f"{len(self._joint_names)} 不一致"
            )
            return
        state = ", ".join(
            f"{name}={value:.2f}"
            for name, value in zip(self._joint_names, message.data, strict=True)
        )
        self.get_logger().info(state)


def main(args: list[str] | None = None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    parsed = _parse_args(remove_ros_args(raw_args)[1:])
    if parsed.print_interval <= 0.0:
        raise SystemExit("--print-interval 必须大于 0")
    rclpy.init(args=raw_args)
    node = StateMonitor(
        parsed.names_topic, parsed.angles_topic, parsed.print_interval
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
