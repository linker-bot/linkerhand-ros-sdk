import math
import time
from dataclasses import dataclass, field

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from linkerbot_hand import (
    CommandKind,
    CommandStage,
    HandDevice,
    HandSide,
    SensorType,
    get_model_capabilities,
)
from linkerbot_hand.testing import FakeBackendFactory, wait_until

from linkerbot_hand_ros.adapter import HandAdapter
from linkerbot_hand_ros.node import LinkerbotHandNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FailingPublisher:
    def publish(self, _message):
        raise RuntimeError("publisher context is invalid")


@dataclass
class NodeRig:
    node: LinkerbotHandNode
    backend_factory: FakeBackendFactory
    adapter_calls: list[dict] = field(default_factory=list)


def make_node_rig(
    *,
    angles_hz=60.0,
    command_rate_hz=100.0,
    transport_options_json="{}",
) -> NodeRig:
    capabilities = get_model_capabilities("L6", side=HandSide.LEFT)
    backend_factory = FakeBackendFactory(capabilities)
    backend_factory.state.set_reading(
        SensorType.ANGLES,
        (100.0, None, 80.0, 70.0, 60.0, 50.0),
        raw_values=(255, 255, 204, 178, 153, 128),
    )
    adapter_calls = []

    def adapter_factory(model, **kwargs):
        adapter_calls.append({"model": model, **kwargs})
        device = HandDevice(
            capabilities,
            backend_factory,
            config=kwargs["session_config"],
        )
        return HandAdapter(model, side=kwargs["side"], device=device)

    node = LinkerbotHandNode(
        adapter_factory=adapter_factory,
        parameter_overrides=[
            Parameter("transport_options_json", value=transport_options_json),
            Parameter("command_rate_hz", value=command_rate_hz),
            Parameter("angles_hz", value=angles_hz),
            Parameter("force_hz", value=0.0),
            Parameter("temperature_hz", value=0.0),
            Parameter("current_hz", value=0.0),
            Parameter("faults_hz", value=0.0),
        ],
    )
    return NodeRig(node, backend_factory, adapter_calls)


@pytest.fixture
def node_rig():
    rclpy.init()
    rig = make_node_rig()
    try:
        yield rig
    finally:
        rig.node.destroy_node()
        rclpy.try_shutdown()


def test_configure_creates_device_configuration_without_io(node_rig):
    result = node_rig.node.trigger_configure()

    assert result == TransitionCallbackReturn.SUCCESS
    assert node_rig.backend_factory.state.open_count == 0
    assert node_rig.backend_factory.state.io_count == 0
    assert node_rig.node._adapter is not None
    assert (
        node_rig.node._adapter.joint_names == get_model_capabilities("L6").joint_names
    )
    assert node_rig.adapter_calls[0]["transport_options"] == {
        "interface_name": "can0",
        "interface_type": "socketcan",
    }
    assert node_rig.node._command_subscriptions == []


def test_activate_opens_device_starts_sdk_polling_and_exposes_commands(node_rig):
    node = node_rig.node
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS

    result = node.trigger_activate()
    wait_until(lambda: len(node_rig.backend_factory.state.poll_log) >= 1)

    assert result == TransitionCallbackReturn.SUCCESS
    assert node_rig.backend_factory.state.open_count == 1
    assert node_rig.backend_factory.state.active_connections == 1
    assert node._active
    assert len(node._command_subscriptions) == 3
    assert node._state_timer is not None
    assert not node._state_timer.is_canceled()


def test_activate_failure_closes_candidate_and_allows_retry():
    rclpy.init()
    rig = make_node_rig(angles_hz=0.0)
    try:
        assert rig.node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        rig.backend_factory.state.disconnect()

        assert rig.node.trigger_activate() == TransitionCallbackReturn.FAILURE
        assert not rig.node._active
        assert rig.node._command_subscriptions == []
        assert rig.backend_factory.state.active_connections == 0

        rig.backend_factory.state.reconnect()
        assert rig.node.trigger_activate() == TransitionCallbackReturn.SUCCESS
        assert rig.backend_factory.state.active_connections == 1
    finally:
        rig.node.destroy_node()
        rclpy.try_shutdown()


def test_cached_state_publication_never_adds_device_io(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()
    wait_until(lambda: len(node_rig.backend_factory.state.poll_log) >= 1)
    node._adapter.stop_polling()
    io_count = node_rig.backend_factory.state.io_count
    original = node._state_publishers["angles"]
    publisher = FakePublisher()
    node._state_publishers["angles"] = publisher
    try:
        for _ in range(1000):
            node._publish_cached_state()
    finally:
        node._state_publishers["angles"] = original

    assert len(publisher.messages) == 1000
    assert list(publisher.messages[-1].data[:1]) == [100.0]
    assert math.isnan(publisher.messages[-1].data[1])
    assert list(publisher.messages[-1].data[2:]) == [80.0, 70.0, 60.0, 50.0]
    assert node_rig.backend_factory.state.io_count == io_count


def test_command_callback_reports_sdk_acceptance_without_waiting(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()
    original = node._accepted_publishers["angles"]
    publisher = FakePublisher()
    node._accepted_publishers["angles"] = publisher
    message = Float64MultiArray(data=[100.0, 50.0, 100.0, 100.0, 100.0, 20.0])
    try:
        started = time.monotonic()
        node._run_command("angles", message)
        callback_elapsed = time.monotonic() - started
    finally:
        node._accepted_publishers["angles"] = original

    handle = node._latest_handles["angles"]
    assert callback_elapsed < 0.05
    assert len(publisher.messages) == 1
    assert list(publisher.messages[0].data) == list(message.data)
    assert handle.stage in {CommandStage.ACCEPTED, CommandStage.DISPATCHED}
    wait_until(lambda: handle.done)
    assert handle.stage is CommandStage.DISPATCHED
    assert node._metrics.snapshot().command_successes == 1


def test_accepted_feedback_failure_does_not_reclassify_sdk_command(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()
    original = node._accepted_publishers["angles"]
    node._accepted_publishers["angles"] = FailingPublisher()
    try:
        node._run_command(
            "angles", Float64MultiArray(data=[100.0, 50.0, 100.0, 100.0, 100.0, 20.0])
        )
    finally:
        node._accepted_publishers["angles"] = original

    handle = node._latest_handles["angles"]
    wait_until(lambda: handle.done)
    metrics = node._metrics.snapshot()
    assert handle.stage is CommandStage.DISPATCHED
    assert metrics.command_successes == 1
    assert metrics.command_errors == 0
    assert metrics.feedback_errors == 1
    assert metrics.latest_feedback_failed


def test_command_burst_keeps_latest_sdk_target_without_history_queue():
    rclpy.init()
    rig = make_node_rig(angles_hz=0.0, command_rate_hz=10.0)
    try:
        node = rig.node
        node.trigger_configure()
        node.trigger_activate()

        for index in range(250):
            value = float(index % 101)
            node._run_command("angles", Float64MultiArray(data=[value] * 6))

        latest = node._latest_handles["angles"]
        wait_until(lambda: latest.done)

        assert latest.stage is CommandStage.DISPATCHED
        assert latest.command.values == (47.0,) * 6
        assert rig.backend_factory.state.dispatched_commands[-1] == latest.command
        assert len(rig.backend_factory.state.dispatched_commands) < 250
        assert node._adapter.get_health().superseded_commands > 0
    finally:
        rig.node.destroy_node()
        rclpy.try_shutdown()


def test_diagnostics_map_sdk_health_and_cached_freshness(node_rig):
    node = node_rig.node
    node.trigger_configure()
    original = node._diagnostic_publisher
    publisher = FakePublisher()
    node._diagnostic_publisher = publisher
    try:
        node._publish_diagnostics()
        inactive = publisher.messages[-1].status[0]
        assert inactive.level == DiagnosticStatus.WARN
        assert inactive.message == "lifecycle node is inactive"

        node.trigger_activate()
        wait_until(lambda: len(node_rig.backend_factory.state.poll_log) >= 1)
        node._publish_diagnostics()
        operational = publisher.messages[-1].status[0]
    finally:
        node._diagnostic_publisher = original

    values = {item.key: item.value for item in operational.values}
    assert operational.level == DiagnosticStatus.OK
    assert operational.message == "operational"
    assert values["model"] == "L6"
    assert values["connection_state"] == "connected"
    assert values["percentage_convention"] == "100=open, 0=closed"


def test_deactivate_stops_polling_but_cleanup_owns_connection_close(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()
    wait_until(lambda: len(node_rig.backend_factory.state.poll_log) >= 1)

    assert node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
    polls_after_deactivate = len(node_rig.backend_factory.state.poll_log)
    time.sleep(0.03)

    assert len(node_rig.backend_factory.state.poll_log) == polls_after_deactivate
    assert node._command_subscriptions == []
    assert node_rig.backend_factory.state.active_connections == 1

    assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
    assert node_rig.backend_factory.state.active_connections == 0
    assert node_rig.backend_factory.state.close_count == 1


def test_cleanup_retries_close_from_lifecycle_error_path(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()
    node.trigger_deactivate()
    adapter = node._adapter
    real_close = adapter.close
    close_attempts = 0

    def close_once_then_succeed():
        nonlocal close_attempts
        close_attempts += 1
        if close_attempts == 1:
            raise RuntimeError("injected close failure")
        real_close()

    adapter.close = close_once_then_succeed

    assert node.trigger_cleanup() == TransitionCallbackReturn.ERROR
    assert close_attempts == 2
    assert node._adapter is None
    assert node_rig.backend_factory.state.active_connections == 0


def test_read_now_publishes_only_fresh_cache_without_device_io(node_rig):
    node = node_rig.node
    node.trigger_configure()
    inactive = node._read_now(Trigger.Request(), Trigger.Response())
    assert not inactive.success

    node.trigger_activate()
    wait_until(lambda: len(node_rig.backend_factory.state.poll_log) >= 1)
    node._adapter.stop_polling()
    io_count = node_rig.backend_factory.state.io_count
    original = node._state_publishers["angles"]
    publisher = FakePublisher()
    node._state_publishers["angles"] = publisher
    try:
        response = node._read_now(Trigger.Request(), Trigger.Response())
    finally:
        node._state_publishers["angles"] = original

    assert response.success
    assert "no device I/O" in response.message
    assert len(publisher.messages) == 1
    assert node_rig.backend_factory.state.io_count == io_count


def test_clear_faults_service_returns_at_sdk_acceptance_stage(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()
    node._adapter.stop_polling()

    response = node._clear_faults(Trigger.Request(), Trigger.Response())

    assert response.success
    assert "accepted by SDK" in response.message
    assert "asynchronous" in response.message
    handle = node._latest_handles["clear_faults"]
    assert handle.command_id > 0
    wait_until(lambda: handle.done)
    assert any(
        command.kind is CommandKind.CLEAR_FAULTS
        for command in node_rig.backend_factory.state.dispatched_commands
    )
    assert handle.stage is CommandStage.DISPATCHED


def test_invalid_transport_json_fails_configure_before_factory_or_io():
    rclpy.init()
    rig = make_node_rig(transport_options_json="[]")
    try:
        assert rig.node.trigger_configure() == TransitionCallbackReturn.FAILURE
        assert rig.adapter_calls == []
        assert rig.backend_factory.state.io_count == 0
    finally:
        rig.node.destroy_node()
        rclpy.try_shutdown()


def test_partial_ros_entity_creation_is_rolled_back_before_retry():
    rclpy.init()
    rig = make_node_rig(angles_hz=0.0)
    node = rig.node
    original_create_service = node.create_service
    baseline_services = len(tuple(node.services))
    service_calls = 0

    def fail_second_service(*args, **kwargs):
        nonlocal service_calls
        service_calls += 1
        if service_calls == 2:
            raise RuntimeError("injected service creation failure")
        return original_create_service(*args, **kwargs)

    try:
        node.create_service = fail_second_service
        assert node.trigger_configure() == TransitionCallbackReturn.FAILURE
        assert len(tuple(node.services)) == baseline_services
        assert node._driver_services == []
        assert node._state_publishers == {}
        assert node._accepted_publishers == {}

        node.create_service = original_create_service
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_one_hundred_lifecycle_cycles_leak_no_connections():
    rclpy.init()
    rig = make_node_rig(angles_hz=0.0)
    try:
        for _ in range(100):
            assert rig.node.trigger_configure() == TransitionCallbackReturn.SUCCESS
            assert rig.node.trigger_activate() == TransitionCallbackReturn.SUCCESS
            assert rig.node.trigger_deactivate() == TransitionCallbackReturn.SUCCESS
            assert rig.node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS

        assert rig.backend_factory.state.open_count == 100
        assert rig.backend_factory.state.close_count == 100
        assert rig.backend_factory.state.max_active_connections == 1
        assert rig.backend_factory.state.active_connections == 0
        assert rig.backend_factory.state.io_count == 0
    finally:
        rig.node.destroy_node()
        rclpy.try_shutdown()


def test_destroy_node_retries_transient_close_failure_once():
    rclpy.init()
    rig = make_node_rig(angles_hz=0.0)
    node = rig.node
    try:
        node.trigger_configure()
        node.trigger_activate()
        adapter = node._adapter
        real_close = adapter.close
        close_attempts = 0

        def close_once_then_succeed():
            nonlocal close_attempts
            close_attempts += 1
            if close_attempts == 1:
                raise RuntimeError("injected close failure")
            real_close()

        adapter.close = close_once_then_succeed

        node.destroy_node()
        assert close_attempts == 2
        assert node._adapter is None
        assert rig.backend_factory.state.active_connections == 0
    finally:
        if not node._driver_destroyed:
            node.destroy_node()
        rclpy.try_shutdown()
