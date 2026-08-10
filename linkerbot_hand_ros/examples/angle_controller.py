"""Interactively publish angle commands from one persistent ROS 2 node."""

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
        "--joint-names-topic",
        default=None,
        help="兼容参数：关节名称话题",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=5.0,
        help="等待驱动订阅者和 DDS 确认的秒数",
    )
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
    if parsed.wait_timeout <= 0.0:
        raise SystemExit("--wait-timeout 必须大于 0")
    try:
        driver_name = resolve_driver_name(
            parsed.driver_name,
            angle_topic=parsed.topic,
            joint_names_topic=parsed.joint_names_topic,
        )
    except ValueError as error:
        raise SystemExit(str(error)) from error

    rclpy.init(args=raw_args)
    node = Node("linkerbot_hand_angle_controller")
    command_client = HandCommandClient(node, driver_name=driver_name)
    logger = node.get_logger()
    try:
        if not spin_until_ready(node, command_client, parsed.wait_timeout):
            raise SystemExit(f"等待驱动就绪超时: {driver_name}")
        logger.info(
            f"已连接 {command_client.model}，{len(command_client.joint_names)} 个关节: "
            f"{', '.join(command_client.joint_names)}"
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
                if not command_client.is_ready():
                    if not spin_until_ready(
                        node, command_client, parsed.wait_timeout
                    ):
                        raise RuntimeError(f"等待驱动就绪超时: {driver_name}")
                receipt = command_client.send_angles(
                    values,
                    wait_for_dds_ack=True,
                    timeout_sec=parsed.wait_timeout,
                )
            except (RuntimeError, ValueError) as error:
                logger.error(str(error))
                continue

            if receipt.dds_acknowledged:
                logger.info(
                    f"已发送 {len(receipt.values)} 个角度值，DDS 确认耗时 "
                    f"{receipt.dds_ack_latency_ms:.2f} ms"
                )
            else:
                logger.error(f"等待话题订阅者确认超时: {driver_name}")
    except KeyboardInterrupt:
        pass
    finally:
        command_client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
