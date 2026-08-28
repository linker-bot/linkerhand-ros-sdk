"""Print typed raw and percent angles from a running hand driver."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from linkerbot_hand_msgs.msg import HandJointState
from linkerbot_hand_ros.qos import STATE_QOS


def _parse_args(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="监听灵巧手角度状态")
    parser.add_argument(
        "--driver-name",
        default="/linkerbot_hand",
    )
    parser.add_argument("--print-interval", type=float, default=0.5)
    return parser.parse_args(args)


class StateMonitor(Node):
    def __init__(self, driver_name: str, print_interval: float) -> None:
        super().__init__("linkerbot_hand_state_monitor")
        self._print_interval = print_interval
        self._last_print = 0.0
        root = driver_name.rstrip("/")
        self.create_subscription(
            HandJointState,
            f"{root}/state/angles",
            self._on_angles,
            STATE_QOS,
        )

    def _on_angles(self, message: HandJointState) -> None:
        now = time.monotonic()
        if now - self._last_print < self._print_interval:
            return
        self._last_print = now
        if not message.meta.valid:
            self.get_logger().warning("角度样本当前无效")
            return
        values = []
        for index, name in enumerate(message.joint_names):
            percent = (
                f"{message.percent[index]:.2f}%"
                if message.percent_valid[index]
                else "unsupported"
            )
            values.append(f"{name}=raw:{message.raw[index]:.2f},percent:{percent}")
        self.get_logger().info(", ".join(values))


def main(args: list[str] | None = None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    parsed = _parse_args(remove_ros_args(raw_args)[1:])
    if parsed.print_interval <= 0:
        raise SystemExit("--print-interval 必须大于 0")
    if not parsed.driver_name.rstrip("/"):
        raise SystemExit("--driver-name 不能为空")
    rclpy.init(args=raw_args)
    node = StateMonitor(parsed.driver_name, parsed.print_interval)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
