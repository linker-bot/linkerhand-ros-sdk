"""Shared helpers for typed command-line clients."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from linkerbot_hand_msgs.msg import HandCommandStatus
from linkerbot_hand_ros.client import HandCommandClient

DEFAULT_DRIVER_NAME = "/linkerbot_hand"


def resolve_driver_name(driver_name: str) -> str:
    """Normalize one driver root without supporting alternate topic contracts."""

    normalized = driver_name.rstrip("/")
    if not normalized:
        raise ValueError("驱动节点名不能为空")
    return normalized


def spin_until_ready(
    node: Node,
    command_client: HandCommandClient,
    timeout_s: float,
    semantic: str = "angles",
) -> bool:
    """Process metadata callbacks until one typed command is ready."""

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if command_client.is_ready(semantic):
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return command_client.is_ready(semantic)


def spin_until_terminal(
    node: Node,
    command_client: HandCommandClient,
    user_sequence: int,
    timeout_s: float,
) -> HandCommandStatus | None:
    """Spin until SDK protocol completion, not mechanical target arrival."""

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        status = command_client.get_status(user_sequence)
        if status is None:
            continue
        if status.stage in {
            HandCommandStatus.STAGE_ACKNOWLEDGED,
            HandCommandStatus.STAGE_FAILED,
            HandCommandStatus.STAGE_SUPERSEDED,
        } or (
            status.stage == HandCommandStatus.STAGE_DISPATCHED
            and status.ack_support == HandCommandStatus.ACK_UNSUPPORTED
        ):
            return status
    return None


__all__ = [
    "DEFAULT_DRIVER_NAME",
    "resolve_driver_name",
    "spin_until_ready",
    "spin_until_terminal",
]
