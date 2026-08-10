"""Publish one angle command after a driver subscriber is available."""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from linkerbot_hand_ros.client import HandCommandClient
from linkerbot_hand_ros.examples._command_cli import (
    DEFAULT_DRIVER_NAME,
    resolve_driver_name,
    spin_until_ready,
)


def _parse_args(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="发送一次灵巧手角度命令")
    parser.add_argument(
        "--driver-name",
        default=DEFAULT_DRIVER_NAME,
        help="驱动节点完整名称",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="兼容参数：角度命令话题",
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
    try:
        driver_name = resolve_driver_name(
            parsed.driver_name,
            angle_topic=parsed.topic,
        )
    except ValueError as error:
        raise SystemExit(str(error)) from error

    rclpy.init(args=raw_args)
    node = Node("linkerbot_hand_angle_command")
    command_client = HandCommandClient(node, driver_name=driver_name)
    try:
        if not spin_until_ready(node, command_client, parsed.wait_timeout):
            raise SystemExit(f"等待驱动就绪超时: {driver_name}")
        try:
            receipt = command_client.send_angles(
                parsed.values,
                wait_for_dds_ack=True,
                timeout_sec=parsed.wait_timeout,
            )
        except (RuntimeError, ValueError) as error:
            raise SystemExit(str(error)) from error
        if not receipt.dds_acknowledged:
            raise SystemExit(f"等待话题订阅者确认超时: {driver_name}")
        node.get_logger().info(
            f"已向 {driver_name} 发送 {len(receipt.values)} 个角度值"
        )
    finally:
        command_client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
