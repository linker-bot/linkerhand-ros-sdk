"""ROS 2 bridge between O6 slider JointState values and HandJointCommand."""

from __future__ import annotations

import secrets
import threading
import time
from typing import Any

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool

from linkerbot_hand_msgs.msg import HandCapabilities, HandJointCommand, HandJointState

from .asset_resolver import load_o6_decoder, resolve_o6_assets
from .joint_mapping import O6JointMapping

_TARGET_TOPIC = "/linkerbot_hand_gui/raw_target_joint_states"
_INITIAL_TARGET_TOPIC = "/linkerbot_hand_gui/initial_joint_states"
_TARGET_OUTPUT_TOPIC = "/linkerbot_hand_gui/target_joint_states"
_ACTUAL_OUTPUT_TOPIC = "/linkerbot_hand_gui/actual_joint_states"

_COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
_STATE_QOS = _COMMAND_QOS
_METADATA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class O6GuiBridge(Node):
    """Keep GUI state cache-only and let the ROS driver own device I/O."""

    def __init__(self) -> None:
        super().__init__("o6_gui_bridge")
        self.declare_parameter("driver_name", "/linkerbot_hand")
        self.declare_parameter("side", "right")
        self.declare_parameter("assets_root", "")
        self.declare_parameter("control_enabled", False)
        self.declare_parameter("command_rate_hz", 30.0)

        driver_name = str(self.get_parameter("driver_name").value).rstrip("/")
        side = str(self.get_parameter("side").value)
        assets_root = str(self.get_parameter("assets_root").value) or None
        assets = resolve_o6_assets(side, assets_root)
        self._mapping = O6JointMapping.from_assets(assets)
        # Loading the decoder here also validates that the selected asset is the
        # official O6 mapping rather than an unrelated hand model.
        load_o6_decoder(assets)
        self._driver_name = driver_name
        self._side = side
        self._lock = threading.RLock()
        self._capabilities: HandCapabilities | None = None
        self._latest_target: dict[str, float] | None = None
        self._target_revision = 0
        self._target_armed = False
        self._initial_state_seeded = False
        self._initial_state: dict[str, float] | None = None
        self._initial_seed_until = 0.0
        self._last_valid_state = 0.0
        self._connection_generation: int | None = None
        self._sequence = 0
        self._client_id = secrets.randbits(63) or 1
        self._control_enabled = bool(self.get_parameter("control_enabled").value)

        self._actual_publisher = self.create_publisher(
            JointState, _ACTUAL_OUTPUT_TOPIC, 10
        )
        # ``joint_state_publisher_gui`` subscribes to this topic through its
        # ``source_list`` parameter.  The bridge publishes exactly one sample
        # after the first verified device state, so later feedback cannot drag
        # a slider away from the user's target.
        self._initial_target_publisher = self.create_publisher(
            JointState,
            _INITIAL_TARGET_TOPIC,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # ``joint_state_publisher_gui`` uses a volatile subscription for
        # ``source_list``.  A single latched sample can therefore be missed
        # during DDS discovery.  Republish the initial state briefly so the
        # slider process always receives it, then stop permanently.
        self._initial_seed_timer = self.create_timer(0.1, self._publish_initial_seed)
        self._target_publisher = self.create_publisher(
            JointState, _TARGET_OUTPUT_TOPIC, 10
        )
        self._command_publisher = self.create_publisher(
            HandJointCommand,
            f"{driver_name}/command/angles",
            _COMMAND_QOS,
        )
        self.create_subscription(
            HandCapabilities,
            f"{driver_name}/capabilities",
            self._on_capabilities,
            _METADATA_QOS,
        )
        self.create_subscription(
            HandJointState,
            f"{driver_name}/state/angles",
            self._on_state,
            _STATE_QOS,
        )
        self.create_subscription(JointState, _TARGET_TOPIC, self._on_target, 10)
        self.create_service(SetBool, "~/set_control_enabled", self._set_control_enabled)

        rate = float(self.get_parameter("command_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        self._timer = self.create_timer(1.0 / rate, self._publish_latest_target)
        control_state = "enabled" if self._control_enabled else "disabled"
        self.get_logger().info(
            "O6 GUI bridge ready; control is "
            f"{control_state} (call ~/set_control_enabled to change)"
        )

    def _on_capabilities(self, message: HandCapabilities) -> None:
        try:
            names = tuple(joint.name for joint in message.joints)
            self._mapping.validate_capabilities(
                names,
                message.identity.model,
                message.identity.side,
            )
        except ValueError as error:
            # A stale transient-local sample from an old driver can coexist
            # briefly with the active driver.  Reject it without discarding a
            # capability that was already validated for this GUI instance.
            self.get_logger().error(f"拒绝不匹配的 O6 capability: {error}")
            return
        with self._lock:
            self._capabilities = message

    def _on_state(self, message: HandJointState) -> None:
        with self._lock:
            capabilities = self._capabilities
        if capabilities is None:
            return
        if tuple(message.joint_names) != tuple(
            joint.name for joint in capabilities.joints
        ):
            self.get_logger().error("state/angles 关节顺序与 capability 不一致")
            return
        valid = bool(message.meta.valid and not message.meta.stale)
        all_percent_valid = len(message.percent_valid) == len(
            self._mapping.capability_names
        ) and all(message.percent_valid)
        if valid and all_percent_valid:
            values = tuple(float(value) for value in message.percent)
            with self._lock:
                generation = int(message.meta.connection_generation)
                if (
                    self._connection_generation is not None
                    and generation != self._connection_generation
                ):
                    self._disarm_target_locked()
                self._connection_generation = generation
                actual = self._mapping.urdf_from_percent(values)
                self._last_valid_state = time.monotonic()
                seed_initial = not self._initial_state_seeded
                if seed_initial:
                    self._initial_state_seeded = True
                    self._initial_state = actual
                    self._initial_seed_until = time.monotonic() + 3.0
            self._publish_joint_state(
                self._actual_publisher,
                actual,
                message.meta.collected_at,
            )
            if seed_initial:
                self._publish_joint_state(
                    self._initial_target_publisher,
                    actual,
                    message.meta.collected_at,
                )

    def _publish_initial_seed(self) -> None:
        """Bridge the first verified state to the GUI during DDS discovery."""

        with self._lock:
            values = None if self._initial_state is None else dict(self._initial_state)
            active = time.monotonic() < self._initial_seed_until
        if values is None:
            return
        if active:
            self._publish_joint_state(
                self._initial_target_publisher,
                values,
                self.get_clock().now().to_msg(),
            )
        else:
            self._initial_seed_timer.cancel()

    def _on_target(self, message: JointState) -> None:
        values = {
            name: float(value)
            for name, value in zip(message.name, message.position, strict=False)
            if name in self._mapping.urdf_names
        }
        if len(values) != len(self._mapping.urdf_names):
            return
        try:
            self._mapping.percent_from_urdf(values)
        except ValueError as error:
            self.get_logger().warning(f"忽略无效 O6 目标: {error}")
            return
        with self._lock:
            # ``joint_state_publisher_gui`` emits a stream even when a slider
            # is stationary.  Republish every valid sample so a late-joining
            # robot_state_publisher (or a transient DDS discovery gap) still
            # receives a current target.  Device commands remain separately
            # rate-limited by ``_publish_latest_target`` and latest-wins, so
            # this does not create a motion history or increase bus traffic.
            self._latest_target = values
            self._target_revision += 1
            if self._control_enabled:
                self._target_armed = True
            target = JointState(
                header=message.header,
                name=list(self._mapping.urdf_names),
                position=[values[name] for name in self._mapping.urdf_names],
            )
        self._target_publisher.publish(target)

    def _publish_latest_target(self) -> None:
        with self._lock:
            target = None if self._latest_target is None else dict(self._latest_target)
            capabilities = self._capabilities
            enabled = self._control_enabled
            state_age = time.monotonic() - self._last_valid_state
            armed = self._target_armed
        if not enabled or not armed or target is None or capabilities is None:
            return
        if not capabilities.identity.verified or state_age > 2.0:
            if state_age > 2.0:
                with self._lock:
                    self._disarm_target_locked()
            return
        if self._command_publisher.get_subscription_count() == 0:
            return
        try:
            percent = self._mapping.percent_from_urdf(target)
        except ValueError as error:
            self.get_logger().warning(f"忽略无效 O6 目标: {error}")
            return
        with self._lock:
            self._sequence += 1
            sequence = self._sequence
        now = self.get_clock().now().to_msg()
        self._command_publisher.publish(
            HandJointCommand(
                client_id=self._client_id,
                user_sequence=sequence,
                publisher_stamp=now,
                representation=HandJointCommand.REPRESENTATION_PERCENT,
                values=list(percent),
            )
        )

    def _set_control_enabled(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        with self._lock:
            if request.data:
                if (
                    self._capabilities is None
                    or not self._capabilities.identity.verified
                ):
                    response.success = False
                    response.message = "等待已验证的 O6 capabilities"
                    return response
                if self._command_publisher.get_subscription_count() == 0:
                    response.success = False
                    response.message = "O6 Lifecycle 驱动未激活"
                    return response
                self._control_enabled = True
                # Do not replay a preview target selected before explicit arming.
                self._target_armed = False
            else:
                self._control_enabled = False
                self._disarm_target_locked()
            response.success = True
            response.message = "实机控制已启用" if request.data else "实机控制已停用"
            return response

    def _disarm_target_locked(self) -> None:
        """Drop the latest motion target while ``self._lock`` is held."""

        self._target_armed = False
        self._latest_target = None

    @staticmethod
    def _publish_joint_state(
        publisher: Any, values: dict[str, float], stamp: Any
    ) -> None:
        publisher.publish(
            JointState(
                header=Header(stamp=stamp),
                name=list(values),
                position=[values[name] for name in values],
            )
        )


def main() -> None:
    import rclpy

    rclpy.init()
    node = O6GuiBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        # A launch-system SIGINT may already have shut down the default
        # context.  Shutdown is idempotent from the application's perspective
        # but rclpy raises in this race, obscuring the original exit signal.
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ["O6GuiBridge", "main"]
