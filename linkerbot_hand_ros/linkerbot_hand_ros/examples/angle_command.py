"""Publish one typed angle command and report its SDK delivery stage."""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from linkerbot_hand_msgs.msg import HandCommandStatus
from linkerbot_hand_ros.client import HandCommandClient
from linkerbot_hand_ros.examples._command_cli import (
    DEFAULT_DRIVER_NAME,
    resolve_driver_name,
    spin_until_ready,
    spin_until_terminal,
)


def _parse_args(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="发送一次灵巧手角度命令")
    parser.add_argument("--driver-name", default=DEFAULT_DRIVER_NAME)
    parser.add_argument("--values", type=float, nargs="+", required=True)
    parser.add_argument(
        "--raw",
        action="store_true",
        help="将 values 解释为精确协议原值；默认解释为 percent",
    )
    parser.add_argument("--wait-timeout", type=float, default=5.0)
    return parser.parse_args(args)


def main(args: list[str] | None = None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    parsed = _parse_args(remove_ros_args(raw_args)[1:])
    if parsed.wait_timeout <= 0:
        raise SystemExit("--wait-timeout 必须大于 0")
    try:
        driver_name = resolve_driver_name(parsed.driver_name)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    rclpy.init(args=raw_args)
    node = Node("linkerbot_hand_angle_command")
    client = HandCommandClient(node, driver_name=driver_name)
    try:
        if not spin_until_ready(node, client, parsed.wait_timeout):
            raise SystemExit(f"等待驱动就绪超时: {driver_name}")
        try:
            publication = (
                client.set_angles(parsed.values)
                if parsed.raw
                else client.set_percent_angles(parsed.values)
            )
        except (RuntimeError, ValueError) as error:
            raise SystemExit(str(error)) from error
        status = spin_until_terminal(
            node,
            client,
            publication.user_sequence,
            parsed.wait_timeout,
        )
        if status is None:
            raise SystemExit("等待 SDK 命令状态超时")
        if status.stage in {
            HandCommandStatus.STAGE_FAILED,
            HandCommandStatus.STAGE_SUPERSEDED,
        }:
            detail = status.error_message or "命令未发送"
            raise SystemExit(f"SDK 命令未完成: {detail}")
        node.get_logger().info(
            f"命令 {status.command_id} 已到达协议阶段 {status.stage}"
        )
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
