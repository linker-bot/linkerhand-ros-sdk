"""Persistent command client for long-running ROS 2 business nodes."""

from __future__ import annotations

import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import Float64MultiArray, String

from .adapter import MODEL_SPECS, validate_command
from .qos import COMMAND_QOS, METADATA_QOS


COMMAND_NAMES = ("angles", "speeds", "torques")


@dataclass(frozen=True)
class CommandReceipt:
    """Result of handing one command to the ROS middleware."""

    sequence: int
    command: str
    values: tuple[float, ...]
    dds_acknowledged: bool | None
    dds_ack_latency_ms: float | None


class HandCommandClient:
    """Own persistent publishers and metadata subscriptions for one hand driver."""

    def __init__(self, node: Node, driver_name: str = "/linkerbot_hand") -> None:
        driver_name = driver_name.rstrip("/")
        if not driver_name:
            raise ValueError("driver_name must not be empty")

        self._node = node
        self._driver_name = driver_name
        self._condition = threading.Condition()
        self._operation_lock = threading.RLock()
        self._joint_names: tuple[str, ...] = ()
        self._model: str | None = None
        self._last_accepted: dict[str, tuple[float, ...] | None] = {
            command: None for command in COMMAND_NAMES
        }
        self._sequence = 0
        self._destroyed = False

        self._publishers: dict[str, Publisher] = {
            command: node.create_publisher(
                Float64MultiArray,
                self._topic(f"command/{command}"),
                COMMAND_QOS,
            )
            for command in COMMAND_NAMES
        }
        self._subscriptions: list[Subscription] = [
            node.create_subscription(
                String,
                self._topic("joint_names"),
                self._update_joint_names,
                METADATA_QOS,
            ),
            node.create_subscription(
                String,
                self._topic("model"),
                self._update_model,
                METADATA_QOS,
            ),
        ]
        for command in COMMAND_NAMES:
            self._subscriptions.append(
                node.create_subscription(
                    Float64MultiArray,
                    self._topic(f"command/accepted/{command}"),
                    lambda message, name=command: self._update_accepted(
                        name, message
                    ),
                    METADATA_QOS,
                )
            )

    def _topic(self, suffix: str) -> str:
        return f"{self._driver_name}/{suffix}"

    def _update_joint_names(self, message: String) -> None:
        names = tuple(
            name.strip() for name in message.data.split(",") if name.strip()
        )
        with self._condition:
            self._joint_names = names
            self._condition.notify_all()

    def _update_model(self, message: String) -> None:
        with self._condition:
            self._model = message.data.strip().lower() or None
            self._condition.notify_all()

    def _update_accepted(
        self, command: str, message: Float64MultiArray
    ) -> None:
        with self._condition:
            self._last_accepted[command] = tuple(float(value) for value in message.data)
            self._condition.notify_all()

    @staticmethod
    def _require_command(command: str) -> None:
        if command not in COMMAND_NAMES:
            raise ValueError(f"unsupported command: {command}")

    def _require_active(self) -> None:
        if self._destroyed:
            raise RuntimeError("hand command client has been destroyed")

    @property
    def driver_name(self) -> str:
        return self._driver_name

    @property
    def model(self) -> str | None:
        with self._condition:
            return self._model

    @property
    def joint_names(self) -> tuple[str, ...]:
        with self._condition:
            return self._joint_names

    def is_ready(self, command: str = "angles") -> bool:
        with self._operation_lock:
            self._require_active()
            self._require_command(command)
            with self._condition:
                model = self._model
                metadata_ready = (
                    model is not None
                    and model in MODEL_SPECS
                    and self._joint_names == MODEL_SPECS[model].joint_names
                )
            return (
                metadata_ready
                and self._publishers[command].get_subscription_count() > 0
            )

    def wait_until_ready(
        self, timeout_sec: float, command: str = "angles"
    ) -> bool:
        """Wait while another executor thread processes discovery callbacks."""
        if timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        deadline = time.monotonic() + timeout_sec
        while not self.is_ready(command):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            with self._condition:
                self._condition.wait(timeout=min(remaining, 0.05))
        return True

    def send(
        self,
        command: str,
        values: Sequence[float],
        *,
        wait_for_dds_ack: bool = False,
        timeout_sec: float = 1.0,
    ) -> CommandReceipt:
        """Publish one command without recreating ROS entities."""
        with self._operation_lock:
            self._require_active()
            self._require_command(command)
            if timeout_sec <= 0.0:
                raise ValueError("timeout_sec must be positive")
            if not self.is_ready(command):
                raise RuntimeError(
                    f"hand driver is not ready for {command}: {self._driver_name}"
                )

            model = self.model
            if model is None:  # Guarded by is_ready(), retained for typing.
                raise RuntimeError("hand model metadata is unavailable")
            normalized = validate_command(model, values, command)
            message = Float64MultiArray()
            message.data = [float(value) for value in normalized]

            self._sequence += 1
            sequence = self._sequence

            acknowledged: bool | None = None
            latency_ms: float | None = None
            if wait_for_dds_ack:
                started_ns = time.monotonic_ns()
                self._publishers[command].publish(message)
                acknowledged = self._publishers[command].wait_for_all_acked(
                    timeout=Duration(seconds=timeout_sec)
                )
                latency_ms = (time.monotonic_ns() - started_ns) / 1_000_000.0
            else:
                self._publishers[command].publish(message)

            return CommandReceipt(
                sequence=sequence,
                command=command,
                values=tuple(float(value) for value in normalized),
                dds_acknowledged=acknowledged,
                dds_ack_latency_ms=latency_ms,
            )

    def send_angles(
        self,
        values: Sequence[float],
        *,
        wait_for_dds_ack: bool = False,
        timeout_sec: float = 1.0,
    ) -> CommandReceipt:
        return self.send(
            "angles",
            values,
            wait_for_dds_ack=wait_for_dds_ack,
            timeout_sec=timeout_sec,
        )

    def send_speeds(
        self,
        values: Sequence[float],
        *,
        wait_for_dds_ack: bool = False,
        timeout_sec: float = 1.0,
    ) -> CommandReceipt:
        return self.send(
            "speeds",
            values,
            wait_for_dds_ack=wait_for_dds_ack,
            timeout_sec=timeout_sec,
        )

    def send_torques(
        self,
        values: Sequence[float],
        *,
        wait_for_dds_ack: bool = False,
        timeout_sec: float = 1.0,
    ) -> CommandReceipt:
        return self.send(
            "torques",
            values,
            wait_for_dds_ack=wait_for_dds_ack,
            timeout_sec=timeout_sec,
        )

    def last_accepted(self, command: str) -> tuple[float, ...] | None:
        with self._operation_lock:
            self._require_active()
            self._require_command(command)
            with self._condition:
                return self._last_accepted[command]

    def destroy(self) -> None:
        with self._operation_lock:
            if self._destroyed:
                return
            self._destroyed = True
            for subscription in self._subscriptions:
                self._node.destroy_subscription(subscription)
            for publisher in self._publishers.values():
                self._node.destroy_publisher(publisher)
            self._subscriptions.clear()
            self._publishers.clear()
