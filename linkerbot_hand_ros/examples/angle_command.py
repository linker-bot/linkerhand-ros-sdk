"""Publish one angle command after a driver subscriber is available."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray


def _parse_args(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="发送一次灵巧手角度命令")
    parser.add_argument(
        "--topic",
        default="/linkerbot_hand/command/angles",
        help="角度命令话题",
    )
    parser.add_argument(
        "--values",
        type=float,
        nargs="+",
        required=True,
        help="按照 joint_names 顺序排列的 0-100 角度值",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=5.0,
        help="等待驱动订阅者的秒数",
    )
    return parser.parse_args(args)


def main(args: list[str] | None = None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    parsed = _parse_args(remove_ros_args(raw_args)[1:])
    if parsed.wait_timeout <= 0.0:
        raise SystemExit("--wait-timeout 必须大于 0")
    if any(value < 0.0 or value > 100.0 for value in parsed.values):
        raise SystemExit("角度值必须在 0 到 100 之间")

    rclpy.init(args=raw_args)
    node = Node("linkerbot_hand_angle_command")
    publisher = node.create_publisher(Float64MultiArray, parsed.topic, 10)
    deadline = time.monotonic() + parsed.wait_timeout
    try:
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise SystemExit(f"等待话题订阅者超时: {parsed.topic}")
        message = Float64MultiArray()
        message.data = parsed.values
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().info(f"已向 {parsed.topic} 发送 {len(parsed.values)} 个角度值")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
