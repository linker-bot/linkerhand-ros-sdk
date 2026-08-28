"""In-process multi-device lifecycle and ROS parameter tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import pytest
import rclpy
from fakes_v1 import DeviceBuilder
from linkerbot_hand_ros.multi_device import (
    HandLifecycleGroup,
    MultiDeviceConfiguration,
    MultiDeviceLifecycleError,
    MultiHandContainerNode,
    create_hand_nodes,
    destroy_hand_nodes,
)
from linkerbot_hand_ros.node import LinkerbotHandNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

PACKAGE_ROOT = Path(__file__).parents[1]


@dataclass
class FakeManagedHandNode:
    namespace: str
    events: list[str]
    results: dict[str, TransitionCallbackReturn] = field(default_factory=dict)
    destroyed: bool = False

    def get_fully_qualified_name(self) -> str:
        return f"{self.namespace}/linkerbot_hand"

    def trigger_configure(self) -> TransitionCallbackReturn:
        return self._transition("configure")

    def trigger_activate(self) -> TransitionCallbackReturn:
        return self._transition("activate")

    def trigger_deactivate(self) -> TransitionCallbackReturn:
        return self._transition("deactivate")

    def trigger_cleanup(self) -> TransitionCallbackReturn:
        return self._transition("cleanup")

    def destroy_node(self) -> None:
        self.events.append(f"destroy:{self.namespace}")
        self.destroyed = True

    def _transition(self, operation: str) -> TransitionCallbackReturn:
        self.events.append(f"{operation}:{self.namespace}")
        return self.results.get(operation, TransitionCallbackReturn.SUCCESS)


def _fake_nodes(
    events: list[str],
) -> tuple[FakeManagedHandNode, FakeManagedHandNode]:
    return (
        FakeManagedHandNode("/left", events),
        FakeManagedHandNode("/right", events),
    )


def test_lifecycle_group_starts_in_order_and_stops_in_reverse_order() -> None:
    events: list[str] = []
    nodes = _fake_nodes(events)
    lifecycle = HandLifecycleGroup(nodes)

    lifecycle.start()
    errors = lifecycle.stop()

    assert errors == ()
    assert events == [
        "configure:/left",
        "configure:/right",
        "activate:/left",
        "activate:/right",
        "deactivate:/right",
        "deactivate:/left",
        "cleanup:/right",
        "cleanup:/left",
    ]
    assert lifecycle.stop() == ()


def test_activate_failure_rolls_back_every_configured_node() -> None:
    events: list[str] = []
    left, right = _fake_nodes(events)
    right.results["activate"] = TransitionCallbackReturn.FAILURE

    with pytest.raises(
        MultiDeviceLifecycleError,
        match=r"/right/linkerbot_hand activate returned FAILURE",
    ):
        HandLifecycleGroup((left, right)).start()

    assert events == [
        "configure:/left",
        "configure:/right",
        "activate:/left",
        "activate:/right",
        "deactivate:/left",
        "cleanup:/right",
        "cleanup:/left",
    ]


def test_configure_failure_cleans_only_nodes_that_completed_configure() -> None:
    events: list[str] = []
    left, right = _fake_nodes(events)
    right.results["configure"] = TransitionCallbackReturn.ERROR

    with pytest.raises(
        MultiDeviceLifecycleError,
        match=r"/right/linkerbot_hand configure returned ERROR",
    ):
        HandLifecycleGroup((left, right)).start()

    assert events == [
        "configure:/left",
        "configure:/right",
        "cleanup:/left",
    ]


def test_partial_node_construction_is_destroyed_in_reverse_order() -> None:
    events: list[str] = []
    created: list[FakeManagedHandNode] = []
    configuration = MultiDeviceConfiguration(
        device_namespaces=("/left", "/right", "/third"),
        autostart=True,
        executor_threads=4,
    )

    def factory(*, namespace: str):
        if namespace == "/third":
            raise RuntimeError("construction failed")
        node = FakeManagedHandNode(namespace, events)
        created.append(node)
        return node

    with pytest.raises(RuntimeError, match="construction failed"):
        create_hand_nodes(configuration, node_factory=factory)

    assert events == ["destroy:/right", "destroy:/left"]
    assert all(node.destroyed for node in created)


def test_two_real_lifecycle_nodes_release_both_fake_devices() -> None:
    rclpy.init()
    builders = {
        "/left": DeviceBuilder(),
        "/right": DeviceBuilder(),
    }
    nodes = tuple(
        LinkerbotHandNode(device_factory=builders[namespace], namespace=namespace)
        for namespace in builders
    )
    lifecycle = HandLifecycleGroup(nodes)
    try:
        lifecycle.start()

        assert all(
            builder.latest_runtime.open_count == 1 for builder in builders.values()
        )
        assert all(
            builder.latest_runtime.polling_plans[-1].angles_hz == 100.0
            for builder in builders.values()
        )

        assert lifecycle.stop() == ()
        assert all(
            builder.latest_runtime.close_count == 1 for builder in builders.values()
        )
    finally:
        lifecycle.stop()
        destroy_hand_nodes(nodes)
        rclpy.try_shutdown()


def test_container_normalizes_namespaces_and_computes_thread_count() -> None:
    rclpy.init()
    container = MultiHandContainerNode(
        parameter_overrides=[
            Parameter("device_namespaces", value=["left", "/right/"]),
            Parameter("autostart", value=False),
        ]
    )
    try:
        configuration = container.read_configuration()
    finally:
        container.destroy_node()
        rclpy.try_shutdown()

    assert configuration == MultiDeviceConfiguration(
        device_namespaces=("/left", "/right"),
        autostart=False,
        executor_threads=4,
    )


def test_container_rejects_duplicate_canonical_namespaces() -> None:
    rclpy.init()
    container = MultiHandContainerNode(
        parameter_overrides=[
            Parameter("device_namespaces", value=["left", "/left"]),
        ]
    )
    try:
        with pytest.raises(ValueError, match="duplicate hand namespace: /left"):
            container.read_configuration()
    finally:
        container.destroy_node()
        rclpy.try_shutdown()


def test_container_rejects_root_namespace() -> None:
    rclpy.init()
    container = MultiHandContainerNode(
        parameter_overrides=[
            Parameter("device_namespaces", value=["/"]),
        ]
    )
    try:
        with pytest.raises(
            ValueError,
            match="the root namespace cannot identify a hand device",
        ):
            container.read_configuration()
    finally:
        container.destroy_node()
        rclpy.try_shutdown()


def test_example_parameter_file_targets_each_in_process_driver() -> None:
    config_path = PACKAGE_ROOT / "config/multi_hand.yaml"
    rclpy.init(args=["--ros-args", "--params-file", str(config_path)])
    container = MultiHandContainerNode()
    nodes = ()
    try:
        configuration = container.read_configuration()
        nodes = create_hand_nodes(configuration)

        assert [node.get_fully_qualified_name() for node in nodes] == [
            "/l6_left/linkerbot_hand",
            "/o6_right/linkerbot_hand",
        ]
        assert [node.get_parameter("model").value for node in nodes] == ["l6", "o6"]
        assert [node.get_parameter("side").value for node in nodes] == [
            "left",
            "right",
        ]
        assert {node.get_parameter("transport_json").value for node in nodes} == {
            '{"backend":"socketcan","channel":"can0"}'
        }
    finally:
        destroy_hand_nodes(nodes)
        container.destroy_node()
        rclpy.try_shutdown()
