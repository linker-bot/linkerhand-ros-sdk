"""Lifecycle and callback contracts for the HandDevice 1.0 ROS path."""

from __future__ import annotations

import threading
import time
from dataclasses import replace

import pytest
import rclpy
from builtin_interfaces.msg import Time
from fakes_v1 import DeviceBuilder
from linkerbot_hand import (
    AngleSample,
    CommandSemantic,
    DataOrigin,
    LifecycleError,
    SampleMeta,
    TelemetrySemantic,
    UnsupportedFirmwareError,
)
from linkerbot_hand_ros.node import LinkerbotHandNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from linkerbot_hand_msgs.msg import HandJointCommand


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class FailOncePublisher(FakePublisher):
    def publish(self, message: object) -> None:
        self.messages.append(message)
        if len(self.messages) == 1:
            raise RuntimeError("status publication failed")


class LockProbePublisher(FakePublisher):
    def __init__(self, lock) -> None:
        super().__init__()
        self._lock = lock
        self.lock_was_available = False

    def publish(self, message: object) -> None:
        self.lock_was_available = self._lock.acquire(blocking=False)
        if self.lock_was_available:
            self._lock.release()
        super().publish(message)


class BlockingPublisher(FakePublisher):
    def __init__(self) -> None:
        super().__init__()
        self.started = threading.Event()
        self.release = threading.Event()

    def publish(self, message: object) -> None:
        self.started.set()
        assert self.release.wait(timeout=1.0)
        super().publish(message)


@pytest.fixture
def node_rig():
    rclpy.init()
    builder = DeviceBuilder()
    node = LinkerbotHandNode(device_factory=builder)
    try:
        yield node, builder
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_configure_constructs_no_runtime_io_or_worker_thread(node_rig):
    node, builder = node_rig
    threads_before = {thread.ident for thread in threading.enumerate()}

    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS

    assert len(builder.calls) == 1
    assert builder.runtime_factories[0].calls == 0
    assert not node._interfaces.management.worker_active
    assert {thread.ident for thread in threading.enumerate()} == threads_before


def test_control_and_observation_entities_use_isolated_callback_groups(node_rig):
    node, _builder = node_rig

    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS

    interfaces = node._interfaces
    commands = interfaces.commands
    management = interfaces.management
    observation_groups = (
        interfaces._state_callback_group,
        interfaces._capability_callback_group,
        interfaces._status_callback_group,
        interfaces._diagnostic_callback_group,
        management._callback_group,
    )
    assert commands._callback_group is node.default_callback_group
    assert all(group is not node.default_callback_group for group in observation_groups)
    groups = (commands._callback_group, *observation_groups)
    assert len({id(group) for group in groups}) == len(groups)
    assert interfaces.state_timer.callback_group is interfaces._state_callback_group
    assert (
        interfaces.capability_timer.callback_group
        is interfaces._capability_callback_group
    )
    assert interfaces.status_timer.callback_group is interfaces._status_callback_group
    assert (
        interfaces.diagnostic_timer.callback_group
        is interfaces._diagnostic_callback_group
    )

    assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
    assert commands._subscriptions
    assert all(
        subscription.callback_group is commands._callback_group
        for subscription in commands._subscriptions
    )


def test_lifecycle_opens_on_activate_and_closes_only_on_cleanup(node_rig):
    node, builder = node_rig
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
    runtime = builder.latest_runtime

    assert runtime.open_count == 1
    assert runtime.polling_plans[-1].angles_hz == 100.0
    assert node._interfaces.management.worker_active

    assert node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
    assert runtime.stop_polling_count == 1
    assert runtime.close_count == 0
    assert not node._interfaces.management.worker_active

    assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
    assert runtime.close_count == 1
    assert node._device is None


def test_interface_deactivate_stops_admin_worker_after_command_cleanup_error(
    node_rig,
):
    node, _builder = node_rig
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
    interfaces = node._interfaces
    commands = interfaces.commands
    assert commands is not None
    original_deactivate = commands.deactivate

    def fail_command_cleanup():
        raise RuntimeError("command cleanup failed")

    commands.deactivate = fail_command_cleanup
    try:
        with pytest.raises(LifecycleError, match="active ROS interfaces"):
            interfaces.deactivate()
        assert not interfaces.management.worker_active
    finally:
        commands.deactivate = original_deactivate


def test_activate_firmware_failure_is_closed_and_retryable():
    rclpy.init()
    builder = DeviceBuilder()
    builder.next_open_error = UnsupportedFirmwareError("old firmware")
    node = LinkerbotHandNode(device_factory=builder)
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_activate() == TransitionCallbackReturn.FAILURE
        assert not node._active
        assert not node._interfaces.management.worker_active

        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
        assert builder.runtime_factories[-1].calls == 2
        assert builder.latest_runtime.open_count == 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_cached_state_publishes_new_sequence_once_without_active_read(node_rig):
    node, builder = node_rig
    node.trigger_configure()
    node.trigger_activate()
    runtime = builder.latest_runtime
    metadata = SampleMeta(
        sequence=1,
        collected_at=time.monotonic(),
        age_s=0.001,
        valid=True,
        stale=False,
        connection_generation=1,
        origin=DataOrigin.DEVICE_RESPONSE,
        within_declared_range=True,
    )
    runtime.snapshot = replace(
        runtime.snapshot,
        snapshot_sequence=1,
        angles=AngleSample(
            raw=(255, 128, 204, 178, 153, 0),
            percent=(100.0, 50.2, 80.0, 69.8, 60.0, 0.0),
            physical_rad=(None,) * 6,
            meta=metadata,
        ),
    )
    publisher = FakePublisher()
    state = node._interfaces.snapshot
    original = state._angles_publisher
    state._angles_publisher = publisher
    reads_before = runtime.read_count
    try:
        node._publish_cached_state()
        node._publish_cached_state()
    finally:
        state._angles_publisher = original

    assert len(publisher.messages) == 1
    assert list(publisher.messages[0].raw) == [255.0, 128.0, 204.0, 178.0, 153.0, 0.0]
    assert runtime.read_count == reads_before


def test_command_callback_routes_raw_angles_without_waiting(node_rig):
    node, builder = node_rig
    node.trigger_configure()
    node.trigger_activate()
    runtime = builder.latest_runtime
    command = runtime.capabilities.command(CommandSemantic.ANGLES)
    assert command is not None
    raw_values = [float(command.range_for(index).minimum) for index in range(6)]
    message = HandJointCommand(
        client_id=29,
        user_sequence=17,
        publisher_stamp=Time(sec=12, nanosec=34),
        representation=HandJointCommand.REPRESENTATION_RAW,
        values=raw_values,
    )
    commands = node._interfaces.commands
    original = commands._status_publisher
    publisher = FakePublisher()
    commands._status_publisher = publisher

    try:
        started = time.monotonic()
        commands._handle_angles(message)
        elapsed = time.monotonic() - started

        assert not publisher.messages
        node._publish_command_status()
    finally:
        commands._status_publisher = original

    assert elapsed < 0.05
    assert runtime.submissions[-1] == (
        CommandSemantic.ANGLES,
        tuple(int(value) for value in raw_values),
    )
    assert runtime.read_count == 0
    assert len(publisher.messages) == 1
    assert publisher.messages[0].stage == publisher.messages[0].STAGE_DISPATCHED
    assert publisher.messages[0].client_id == 29
    assert publisher.messages[0].publisher_stamp == Time(sec=12, nanosec=34)
    received = publisher.messages[0].driver_received_stamp
    accepted = publisher.messages[0].transitions[0].stamp
    assert (received.sec, received.nanosec) <= (accepted.sec, accepted.nanosec)
    assert commands.tracked_count == 0


def test_status_publication_failure_does_not_reclassify_accepted_command(node_rig):
    node, builder = node_rig
    node.trigger_configure()
    node.trigger_activate()
    runtime = builder.latest_runtime
    commands = node._interfaces.commands
    original = commands._status_publisher
    publisher = FailOncePublisher()
    commands._status_publisher = publisher
    before = node._interfaces._metrics.snapshot()
    try:
        commands._handle_angles(
            HandJointCommand(
                client_id=1,
                user_sequence=18,
                representation=HandJointCommand.REPRESENTATION_PERCENT,
                values=[100.0] * 6,
            )
        )
        after = node._interfaces._metrics.snapshot()

        assert after.command_successes == before.command_successes + 1
        assert after.command_errors == before.command_errors
        assert runtime.submissions[-1][0] is CommandSemantic.ANGLES
        assert commands.tracked_count == 1
        assert not publisher.messages

        node._publish_command_status()
        assert commands.tracked_count == 1
        node._publish_command_status()
        assert commands.tracked_count == 0
        assert len(publisher.messages) == 2
    finally:
        commands._status_publisher = original


def test_status_publication_does_not_hold_command_tracking_lock(node_rig):
    node, _builder = node_rig
    node.trigger_configure()
    node.trigger_activate()
    commands = node._interfaces.commands
    original = commands._status_publisher
    publisher = LockProbePublisher(commands._lock)
    commands._status_publisher = publisher
    try:
        commands._handle_angles(
            HandJointCommand(
                client_id=1,
                user_sequence=19,
                representation=HandJointCommand.REPRESENTATION_PERCENT,
                values=[100.0] * 6,
            )
        )

        node._publish_command_status()
    finally:
        commands._status_publisher = original

    assert publisher.lock_was_available
    assert len(publisher.messages) == 1


def test_command_deactivate_waits_for_inflight_status_publication(node_rig):
    node, _builder = node_rig
    node.trigger_configure()
    node.trigger_activate()
    commands = node._interfaces.commands
    original = commands._status_publisher
    publisher = BlockingPublisher()
    commands._status_publisher = publisher
    commands._handle_angles(
        HandJointCommand(
            client_id=1,
            user_sequence=20,
            representation=HandJointCommand.REPRESENTATION_PERCENT,
            values=[100.0] * 6,
        )
    )
    publishing = threading.Thread(target=commands.publish_updates)
    deactivated = threading.Event()
    deactivating = threading.Thread(
        target=lambda: (commands.deactivate(), deactivated.set())
    )
    try:
        publishing.start()
        assert publisher.started.wait(timeout=1.0)
        deactivating.start()
        assert not deactivated.wait(timeout=0.05)

        publisher.release.set()
        publishing.join(timeout=1.0)
        deactivating.join(timeout=1.0)
    finally:
        publisher.release.set()
        publishing.join(timeout=1.0)
        deactivating.join(timeout=1.0)
        commands._status_publisher = original

    assert not publishing.is_alive()
    assert not deactivating.is_alive()
    assert deactivated.is_set()
    assert commands.tracked_count == 0
    assert commands.publish_updates() == 0


def test_diagnostics_reads_health_without_snapshot_or_capability_access(node_rig):
    node, builder = node_rig
    node.trigger_configure()
    node.trigger_activate()
    runtime = builder.latest_runtime
    publisher = FakePublisher()
    original = node._interfaces.diagnostic_publisher
    node._interfaces.diagnostic_publisher = publisher
    before = (
        runtime.health_get_count,
        runtime.snapshot_get_count,
        runtime.capabilities_get_count,
    )
    try:
        node._publish_diagnostics()
    finally:
        node._interfaces.diagnostic_publisher = original

    assert len(publisher.messages) == 1
    assert runtime.health_get_count == before[0] + 1
    assert runtime.snapshot_get_count == before[1]
    assert runtime.capabilities_get_count == before[2]


def test_runtime_firmware_degradation_republishes_capability_without_state_io(
    node_rig,
):
    node, builder = node_rig
    node.trigger_configure()
    node.trigger_activate()
    runtime = builder.latest_runtime
    publisher = FakePublisher()
    snapshot = node._interfaces.snapshot
    original = snapshot._capabilities_publisher
    snapshot._capabilities_publisher = publisher
    runtime.capabilities = replace(
        runtime.capabilities,
        firmware_unsupported_telemetry=(TelemetrySemantic.FORCE,),
    )
    before = (
        runtime.snapshot_get_count,
        runtime.health_get_count,
        runtime.capabilities_get_count,
    )
    try:
        node._interfaces.publish_runtime_capabilities()
        node._interfaces.publish_runtime_capabilities()
    finally:
        snapshot._capabilities_publisher = original

    assert len(publisher.messages) == 1
    assert publisher.messages[0].firmware_unsupported_telemetry == ["force"]
    assert runtime.snapshot_get_count == before[0]
    assert runtime.health_get_count == before[1]
    assert runtime.capabilities_get_count == before[2] + 2


def test_parameter_update_contains_runtime_rollback_failures(node_rig, monkeypatch):
    node, _builder = node_rig
    node.trigger_configure()
    node.trigger_activate()
    poll_calls = 0

    def fail_polling_rollback(_plan):
        nonlocal poll_calls
        poll_calls += 1
        if poll_calls > 1:
            raise RuntimeError("polling rollback failed")

    def fail_rate_update(_configuration):
        raise RuntimeError("timer update failed")

    monkeypatch.setattr(node._device, "start_polling", fail_polling_rollback)
    monkeypatch.setattr(node._interfaces, "apply_rates", fail_rate_update)

    result = node._on_set_parameters([Parameter("angles_hz", value=50.0)])

    assert not result.successful
    assert "timer update failed" in result.reason
    assert "polling rollback failed" in result.reason
    assert "runtime rollback failed" in result.reason
    assert node._configuration.polling_plan.angles_hz == 100.0


def test_one_hundred_lifecycle_cycles_leak_no_admin_threads():
    rclpy.init()
    builder = DeviceBuilder()
    node = LinkerbotHandNode(device_factory=builder)
    try:
        for _ in range(100):
            assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
            assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
            assert node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
            assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS

        runtimes = [
            factory.runtime
            for factory in builder.runtime_factories
            if factory.runtime is not None
        ]
        assert len(runtimes) == 100
        assert sum(runtime.open_count for runtime in runtimes) == 100
        assert sum(runtime.close_count for runtime in runtimes) == 100
        assert not any(
            thread.name == "linkerbot_hand-admin" for thread in threading.enumerate()
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
