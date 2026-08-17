import math
import time
from dataclasses import dataclass, field

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from linkerbot_hand import (
    MODEL_CAPABILITIES,
    CommandKind,
    CommandStage,
    HandDevice,
    HandSide,
    SensorType,
    get_model_capabilities,
)
from linkerbot_hand.testing import FakeBackendFactory, wait_until
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

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
    model="L6",
    angles_hz=60.0,
    command_rate_hz=100.0,
    transport_options_json="{}",
) -> NodeRig:
    capabilities = get_model_capabilities(model, side=HandSide.LEFT)
    backend_factory = FakeBackendFactory(capabilities)
    angle_values = (100.0,) * capabilities.joint_count
    raw_angle_values = tuple(
        joint.percentage_mapping.raw_open for joint in capabilities.joints
    )
    if capabilities.model == "L6":
        angle_values = (100.0, None, 80.0, 70.0, 60.0, 50.0)
        raw_angle_values = (255, 255, 204, 178, 153, 128)
    backend_factory.state.set_reading(
        SensorType.ANGLES,
        angle_values,
        raw_values=raw_angle_values,
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
            Parameter("model", value=model),
            Parameter("transport_options_json", value=transport_options_json),
            Parameter("command_rate_hz", value=command_rate_hz),
            Parameter("angles_hz", value=angles_hz),
            Parameter("speeds_hz", value=0.0),
            Parameter("torques_hz", value=0.0),
            Parameter("force_hz", value=0.0),
            Parameter("temperature_hz", value=0.0),
            Parameter("current_hz", value=0.0),
            Parameter("faults_hz", value=0.0),
            Parameter("voltage_hz", value=0.0),
            Parameter("acceleration_hz", value=0.0),
            Parameter("motion_time_hz", value=0.0),
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
    assert len(node._command_subscriptions) == 4
    assert node._state_timer is not None
    assert not node._state_timer.is_canceled()


def test_runtime_sensor_rates_reconfigure_sdk_polling_and_state_topics(node_rig):
    node = node_rig.node
    state = node_rig.backend_factory.state
    state.set_reading(SensorType.TORQUES, (6.0, 5.0, 4.0, 3.0, 2.0, 1.0))
    node.trigger_configure()
    node.trigger_activate()

    result = node.set_parameters_atomically(
        [
            Parameter("speeds_hz", value=30.0),
            Parameter("torques_hz", value=30.0),
        ]
    )
    wait_until(
        lambda: SensorType.TORQUES in state.poll_log
    )
    original_torques = node._state_publishers["torques"]
    torques = FakePublisher()
    node._state_publishers["torques"] = torques
    try:
        node._publish_cached_state()
    finally:
        node._state_publishers["torques"] = original_torques

    assert result.successful
    assert SensorType.SPEEDS not in state.poll_log
    assert list(torques.messages[-1].data) == [6.0, 5.0, 4.0, 3.0, 2.0, 1.0]

    result = node.set_parameters_atomically(
        [
            Parameter("speeds_hz", value=0.0),
            Parameter("torques_hz", value=0.0),
        ]
    )
    torque_polls_after_disable = state.poll_log.count(SensorType.TORQUES)
    time.sleep(0.05)

    original_torques = node._state_publishers["torques"]
    torques = FakePublisher()
    node._state_publishers["torques"] = torques
    try:
        node._publish_cached_state()
    finally:
        node._state_publishers["torques"] = original_torques

    assert result.successful
    assert state.poll_log.count(SensorType.TORQUES) == torque_polls_after_disable
    assert torques.messages == []


@pytest.mark.parametrize("model", MODEL_CAPABILITIES)
def test_runtime_speed_and_torque_polling_reuses_connection_for_every_model(model):
    rclpy.init()
    rig = make_node_rig(model=model, angles_hz=0.0)
    node = rig.node
    state = rig.backend_factory.state
    capabilities = get_model_capabilities(model)
    supported = {
        sensor
        for sensor in (SensorType.SPEEDS, SensorType.TORQUES)
        if capabilities.sensor(sensor) is not None
    }
    try:
        for sensor in supported:
            state.set_reading(sensor, (1.0,) * capabilities.joint_count)

        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS
        generation = node._adapter.get_health().connection_generation

        result = node.set_parameters_atomically(
            [
                Parameter("speeds_hz", value=30.0),
                Parameter("torques_hz", value=30.0),
            ]
        )
        assert result.successful
        for sensor in supported:
            wait_until(lambda sensor=sensor: sensor in state.poll_log)
        for sensor in {SensorType.SPEEDS, SensorType.TORQUES}.difference(supported):
            assert sensor not in state.poll_log

        result = node.set_parameters_atomically(
            [
                Parameter("speeds_hz", value=0.0),
                Parameter("torques_hz", value=0.0),
            ]
        )
        assert result.successful
        polls_after_disable = {
            sensor: state.poll_log.count(sensor) for sensor in supported
        }
        time.sleep(0.05)

        assert all(
            state.poll_log.count(sensor) == count
            for sensor, count in polls_after_disable.items()
        )
        assert node._adapter.get_health().connection_generation == generation
        assert state.open_count == 1
        assert state.active_connections == 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_runtime_publication_rates_pause_resume_and_reject_invalid(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()

    result = node.set_parameters_atomically(
        [
            Parameter("publish_rate_hz", value=0.0),
            Parameter("diagnostic_rate", value=0.0),
        ]
    )
    assert result.successful
    assert node._state_timer.is_canceled()
    assert node._diagnostic_timer.is_canceled()

    result = node.set_parameters_atomically(
        [
            Parameter("publish_rate_hz", value=20.0),
            Parameter("diagnostic_rate", value=2.0),
        ]
    )
    assert result.successful
    assert not node._state_timer.is_canceled()
    assert not node._diagnostic_timer.is_canceled()
    assert node._state_timer.timer_period_ns == 50_000_000
    assert node._diagnostic_timer.timer_period_ns == 500_000_000

    result = node.set_parameters_atomically(
        [Parameter("angles_hz", value=-1.0)]
    )
    assert not result.successful
    assert node.get_parameter("angles_hz").value == 60.0


def test_static_parameters_require_cleanup_before_update(node_rig):
    node = node_rig.node

    unconfigured = node.set_parameters_atomically(
        [Parameter("command_rate_hz", value=80.0)]
    )
    assert unconfigured.successful
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS

    configured = node.set_parameters_atomically(
        [Parameter("command_rate_hz", value=70.0)]
    )
    assert not configured.successful
    assert "require cleanup" in configured.reason
    assert node.get_parameter("command_rate_hz").value == 80.0

    assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
    after_cleanup = node.set_parameters_atomically(
        [Parameter("command_rate_hz", value=70.0)]
    )
    assert after_cleanup.successful
    assert node.get_parameter("command_rate_hz").value == 70.0


def test_unconfigured_parameter_transaction_validates_complete_configuration(node_rig):
    node = node_rig.node

    invalid_model = node.set_parameters_atomically(
        [Parameter("model", value="unknown-hand")]
    )
    invalid_side = node.set_parameters_atomically(
        [Parameter("side", value="up")]
    )
    invalid_transport = node.set_parameters_atomically(
        [Parameter("transport_options_json", value="[]")]
    )

    assert not invalid_model.successful
    assert "unsupported hand model" in invalid_model.reason
    assert not invalid_side.successful
    assert "side must be" in invalid_side.reason
    assert not invalid_transport.successful
    assert "JSON object" in invalid_transport.reason
    assert node.get_parameter("model").value == "L6"
    assert node.get_parameter("side").value == "left"
    assert node.get_parameter("transport_options_json").value == "{}"


def test_o30i_runtime_numeric_sensor_polling_and_publication():
    rclpy.init()
    rig = make_node_rig(model="O30i", angles_hz=0.0)
    node = rig.node
    state = rig.backend_factory.state
    sensor_values = {
        SensorType.VOLTAGE: (24.1,) * 20,
        SensorType.ACCELERATION: (3.0,) * 20,
        SensorType.MOTION_TIME: (12.0,) * 20,
    }
    try:
        for sensor, values in sensor_values.items():
            state.set_reading(sensor, values)
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

        result = node.set_parameters_atomically(
            [
                Parameter("voltage_hz", value=1.0),
                Parameter("acceleration_hz", value=10.0),
                Parameter("motion_time_hz", value=10.0),
            ]
        )
        assert result.successful
        for sensor in sensor_values:
            wait_until(lambda sensor=sensor: sensor in state.poll_log)

        replacements = {}
        originals = {}
        for topic_name in ("voltage", "acceleration", "motion_time"):
            originals[topic_name] = node._state_publishers[topic_name]
            replacements[topic_name] = FakePublisher()
            node._state_publishers[topic_name] = replacements[topic_name]
        try:
            node._publish_cached_state()
        finally:
            node._state_publishers.update(originals)

        assert list(replacements["voltage"].messages[-1].data) == [24.1] * 20
        assert list(replacements["acceleration"].messages[-1].data) == [3.0] * 20
        assert list(replacements["motion_time"].messages[-1].data) == [12.0] * 20
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_inactive_rate_update_applies_on_next_activate(node_rig):
    node = node_rig.node
    state = node_rig.backend_factory.state
    node.trigger_configure()
    node.trigger_activate()
    wait_until(lambda: SensorType.ANGLES in state.poll_log)
    node.trigger_deactivate()

    result = node.set_parameters_atomically(
        [Parameter("angles_hz", value=0.0)]
    )
    state.poll_log.clear()
    node.trigger_activate()
    time.sleep(0.05)

    assert result.successful
    assert SensorType.ANGLES not in state.poll_log

    result = node.set_parameters_atomically(
        [Parameter("angles_hz", value=60.0)]
    )
    wait_until(lambda: SensorType.ANGLES in state.poll_log)

    assert result.successful


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


def test_cached_raw_angle_publication_uses_protocol_values_without_io(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()
    wait_until(lambda: len(node_rig.backend_factory.state.poll_log) >= 1)
    node._adapter.stop_polling()
    io_count = node_rig.backend_factory.state.io_count
    original = node._state_publishers["raw_angles"]
    publisher = FakePublisher()
    node._state_publishers["raw_angles"] = publisher
    try:
        node._publish_cached_state()
    finally:
        node._state_publishers["raw_angles"] = original

    assert len(publisher.messages) == 1
    assert list(publisher.messages[0].data) == [255, 255, 204, 178, 153, 128]
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


def test_raw_angle_callback_submits_protocol_values_without_waiting(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()
    original = node._accepted_publishers["raw_angles"]
    publisher = FakePublisher()
    node._accepted_publishers["raw_angles"] = publisher
    message = Float64MultiArray(data=[255.0, 128.0, 204.0, 178.0, 153.0, 0.0])
    try:
        started = time.monotonic()
        node._run_command("raw_angles", message)
        callback_elapsed = time.monotonic() - started
    finally:
        node._accepted_publishers["raw_angles"] = original

    handle = node._latest_handles["raw_angles"]
    assert callback_elapsed < 0.05
    assert list(publisher.messages[0].data) == list(message.data)
    assert handle.command.values == (255, 128, 204, 178, 153, 0)
    wait_until(lambda: handle.done)
    assert handle.stage is CommandStage.DISPATCHED


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


def test_active_shutdown_stops_worker_and_closes_connection(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()
    wait_until(lambda: len(node_rig.backend_factory.state.poll_log) >= 1)

    assert node.trigger_shutdown() == TransitionCallbackReturn.SUCCESS
    assert not node._active
    assert node._adapter is None
    assert node._configuration is None
    assert node._command_subscriptions == []
    assert node_rig.backend_factory.state.active_connections == 0
    assert node_rig.backend_factory.state.close_count == 1


@pytest.mark.parametrize(
    ("callback_name", "base_callback_name"),
    (
        ("on_cleanup", "on_cleanup"),
        ("on_shutdown", "on_shutdown"),
        ("on_error", "on_error"),
    ),
)
def test_teardown_releases_sdk_when_managed_ros_callback_raises(
    node_rig, monkeypatch, callback_name, base_callback_name
):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()

    def fail_managed_callback(_node, _state):
        raise RuntimeError("injected managed entity failure")

    monkeypatch.setattr(LifecycleNode, base_callback_name, fail_managed_callback)

    result = getattr(node, callback_name)(node._state_machine.current_state)

    assert result == TransitionCallbackReturn.ERROR
    assert node._adapter is None
    assert node._configuration is None
    assert node_rig.backend_factory.state.active_connections == 0
    assert node_rig.backend_factory.state.close_count == 1


def test_deactivate_failure_enters_error_cleanup_and_closes_connection(node_rig):
    node = node_rig.node
    node.trigger_configure()
    node.trigger_activate()
    adapter = node._adapter
    real_stop_polling = adapter.stop_polling
    stop_attempts = 0

    def stop_once_then_succeed():
        nonlocal stop_attempts
        stop_attempts += 1
        if stop_attempts == 1:
            raise RuntimeError("injected stop failure")
        real_stop_polling()

    adapter.stop_polling = stop_once_then_succeed

    assert node.trigger_deactivate() == TransitionCallbackReturn.ERROR
    assert stop_attempts == 2
    assert node._adapter is None
    assert node._configuration is None
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


@pytest.mark.parametrize(
    ("operation", "command_kind"),
    (
        ("enable", CommandKind.ENABLE),
        ("disable", CommandKind.DISABLE),
        ("calibrate", CommandKind.CALIBRATE),
    ),
)
def test_l30_management_services_return_at_sdk_acceptance(operation, command_kind):
    rclpy.init()
    rig = make_node_rig(model="L30", angles_hz=0.0)
    node = rig.node
    try:
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

        response = getattr(node, f"_{operation}")(
            Trigger.Request(), Trigger.Response()
        )

        assert response.success
        assert "accepted by SDK" in response.message
        handle = node._latest_handles[operation]
        wait_until(lambda: handle.done)
        assert handle.command.kind is command_kind
        assert handle.stage is CommandStage.ACKNOWLEDGED
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_unsupported_management_service_reports_model_capability(node_rig):
    node = node_rig.node
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    assert node.trigger_activate() == TransitionCallbackReturn.SUCCESS

    response = node._enable(Trigger.Request(), Trigger.Response())

    assert not response.success
    assert response.message == "device enable is not supported by this model"


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
