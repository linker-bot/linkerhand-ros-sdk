"""Shared helpers for command-line clients."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from linkerbot_hand_ros.client import HandCommandClient


DEFAULT_DRIVER_NAME = "/linkerbot_hand"


def resolve_driver_name(
    driver_name: str,
    *,
    angle_topic: str | None = None,
    joint_names_topic: str | None = None,
) -> str:
    """Resolve legacy topic overrides to one canonical driver name."""
    candidates: set[str] = set()
    for topic, suffix in (
        (angle_topic, "/command/angles"),
        (joint_names_topic, "/joint_names"),
    ):
        if topic is None:
            continue
        normalized = topic.rstrip("/")
        if not normalized.endswith(suffix):
            raise ValueError(f"话题必须以 {suffix} 结尾: {topic}")
        candidates.add(normalized[: -len(suffix)])

    normalized_driver = driver_name.rstrip("/")
    if len(candidates) > 1:
        raise ValueError("命令话题和关节名称话题必须属于同一个驱动节点")
    if candidates:
        topic_driver = candidates.pop()
        if (
            normalized_driver != DEFAULT_DRIVER_NAME
            and normalized_driver != topic_driver
        ):
            raise ValueError("--driver-name 与话题参数指向不同驱动节点")
        normalized_driver = topic_driver
    if not normalized_driver:
        raise ValueError("驱动节点名不能为空")
    return normalized_driver


def spin_until_ready(
    node: Node,
    command_client: HandCommandClient,
    timeout_sec: float,
) -> bool:
    """Process metadata callbacks until the driver contract is ready."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if command_client.is_ready():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return command_client.is_ready()
