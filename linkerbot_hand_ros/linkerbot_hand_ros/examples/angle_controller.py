"""Interactively publish typed angle commands from one persistent ROS node."""

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
    parser = argparse.ArgumentParser(description="持续发送灵巧手角度命令")
    parser.add_argument("--driver-name", default=DEFAULT_DRIVER_NAME)
    parser.add_argument("--raw", action="store_true")
    parser.add_argument("--wait-timeout", type=float, default=5.0)
    return parser.parse_args(args)


def _parse_values(line: str) -> list[float]:
    parts = line.replace(",", " ").split()
    try:
        return [float(part) for part in parts]
    except ValueError as error:
        raise ValueError("角度值必须是数字") from error


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
    node = Node("linkerbot_hand_angle_controller")
    client = HandCommandClient(node, driver_name=driver_name)
    logger = node.get_logger()
    try:
        if not spin_until_ready(node, client, parsed.wait_timeout):
            raise SystemExit(f"等待驱动就绪超时: {driver_name}")
        logger.info(
            f"已连接 {client.model}，{len(client.joint_names)} 个关节: "
            f"{', '.join(client.joint_names)}"
        )
        while rclpy.ok():
            if sys.stdin.isatty():
                print("angles> ", end="", flush=True)
            line = sys.stdin.readline()
            if not line:
                break
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.lower() in {"exit", "quit"}:
                break
            try:
                values = _parse_values(stripped)
                publication = (
                    client.set_angles(values)
                    if parsed.raw
                    else client.set_percent_angles(values)
                )
            except (RuntimeError, ValueError) as error:
                logger.error(str(error))
                continue
            logger.info(
                f"已发布序号 {publication.user_sequence}，"
                f"共 {len(publication.values)} 个角度值"
            )
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
