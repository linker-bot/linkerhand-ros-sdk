"""Interactively publish angle commands from one persistent ROS 2 node."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray, String


def _parse_args(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="持续发送灵巧手角度命令")
    parser.add_argument(
        "--topic",
        default="/linkerbot_hand/command/angles",
        help="角度命令话题",
    )
    parser.add_argument(
        "--joint-names-topic",
        default="/linkerbot_hand/joint_names",
        help="关节名称话题",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=5.0,
        help="等待驱动订阅者和 DDS 确认的秒数",
    )
    return parser.parse_args(args)


def _parse_values(line: str, joint_count: int) -> list[float]:
    parts = line.replace(",", " ").split()
    if len(parts) != joint_count:
        raise ValueError(f"需要 {joint_count} 个角度值，收到 {len(parts)} 个")
    try:
        values = [float(part) for part in parts]
    except ValueError as error:
        raise ValueError("角度值必须是数字") from error
    if any(
        not math.isfinite(value) or value < 0.0 or value > 100.0
        for value in values
    ):
        raise ValueError("角度值必须是 0 到 100 之间的有限数值")
    return values


def _wait_until_ready(
    node: Node,
    publisher: Publisher,
    joint_names: list[str],
    timeout: float,
) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if publisher.get_subscription_count() > 0 and joint_names:
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return publisher.get_subscription_count() > 0 and bool(joint_names)


def main(args: list[str] | None = None) -> None:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    parsed = _parse_args(remove_ros_args(raw_args)[1:])
    if parsed.wait_timeout <= 0.0:
        raise SystemExit("--wait-timeout 必须大于 0")

    rclpy.init(args=raw_args)
    node = Node("linkerbot_hand_angle_controller")
    publisher = node.create_publisher(Float64MultiArray, parsed.topic, 10)
    joint_names: list[str] = []

    def update_joint_names(message: String) -> None:
        joint_names[:] = [
            name.strip() for name in message.data.split(",") if name.strip()
        ]

    node.create_subscription(
        String,
        parsed.joint_names_topic,
        update_joint_names,
        1,
    )

    logger = node.get_logger()
    try:
        if not _wait_until_ready(
            node,
            publisher,
            joint_names,
            parsed.wait_timeout,
        ):
            raise SystemExit("等待驱动订阅者或关节名称超时")
        logger.info(f"已连接 {len(joint_names)} 个关节: {', '.join(joint_names)}")

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
                values = _parse_values(stripped, len(joint_names))
            except ValueError as error:
                logger.error(str(error))
                continue

            subscriber_missing = publisher.get_subscription_count() == 0
            if subscriber_missing and not _wait_until_ready(
                node, publisher, joint_names, parsed.wait_timeout
            ):
                logger.error(f"等待话题订阅者超时: {parsed.topic}")
                continue

            message = Float64MultiArray()
            message.data = values
            started_ns = time.monotonic_ns()
            publisher.publish(message)
            acknowledged = publisher.wait_for_all_acked(
                timeout=Duration(seconds=parsed.wait_timeout)
            )
            elapsed_ms = (time.monotonic_ns() - started_ns) / 1_000_000.0
            if acknowledged:
                logger.info(
                    f"已发送 {len(values)} 个角度值，DDS 确认耗时 {elapsed_ms:.2f} ms"
                )
            else:
                logger.error(f"等待话题订阅者确认超时: {parsed.topic}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
