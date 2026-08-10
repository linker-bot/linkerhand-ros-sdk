import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Float64MultiArray

from linkerbot_hand_ros.node import LinkerbotHandNode


class FakeAdapter:
    model = "l6"
    joint_names = (
        "thumb_flex",
        "thumb_abd",
        "index",
        "middle",
        "ring",
        "pinky",
    )

    def __init__(self):
        self.angle_commands = []
        self.closed = False
        self.command_error = None

    def set_angles(self, values):
        if self.command_error is not None:
            raise self.command_error
        normalized = [float(value) for value in values]
        self.angle_commands.append(normalized)
        return normalized

    def read_angles(self, _timeout_ms):
        return [100.0, 50.0, 100.0, 100.0, 100.0, 100.0]

    def read_optional(self, _name, _timeout_ms):
        return None

    def clear_faults(self):
        return None

    def close(self):
        self.closed = True


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def hand_node():
    rclpy.init()
    adapter = FakeAdapter()

    def factory(*_args, **_kwargs):
        return adapter

    node = LinkerbotHandNode(adapter_factory=factory)
    try:
        yield node, adapter
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_driver_publishes_only_sdk_accepted_commands(hand_node):
    node, adapter = hand_node
    accepted_publisher = FakePublisher()
    node._accepted_publishers["angles"] = accepted_publisher
    message = Float64MultiArray(data=[100.0, 50.0, 100.0, 100.0, 100.0, 20.0])

    node._run_command("angles", message)

    assert adapter.angle_commands == [[100.0, 50.0, 100.0, 100.0, 100.0, 20.0]]
    assert list(accepted_publisher.messages[0].data) == adapter.angle_commands[0]
    assert node._metrics.snapshot().command_successes == 1

    adapter.command_error = RuntimeError("CAN write failed")
    node._run_command("angles", message)

    assert len(accepted_publisher.messages) == 1
    snapshot = node._metrics.snapshot()
    assert snapshot.command_errors == 1
    assert snapshot.latest_command_failed


def test_driver_diagnostics_transition_from_waiting_to_operational(hand_node):
    node, _adapter = hand_node
    diagnostic_publisher = FakePublisher()
    node._diagnostic_publisher = diagnostic_publisher

    node._publish_diagnostics()
    waiting = diagnostic_publisher.messages[-1].status[0]
    assert waiting.level == DiagnosticStatus.WARN
    assert waiting.message == "waiting for first state sample"

    node._metrics.record_state_success()
    node._publish_diagnostics()
    operational = diagnostic_publisher.messages[-1].status[0]
    values = {item.key: item.value for item in operational.values}

    assert operational.level == DiagnosticStatus.OK
    assert operational.message == "operational"
    assert values["model"] == "l6"
    assert values["state_successes"] == "1"


def test_driver_diagnostics_report_initial_read_failures_as_error(hand_node):
    node, _adapter = hand_node
    diagnostic_publisher = FakePublisher()
    node._diagnostic_publisher = diagnostic_publisher

    node._metrics.record_state_error(RuntimeError("CAN read timeout"))
    node._publish_diagnostics()

    status = diagnostic_publisher.messages[-1].status[0]
    values = {item.key: item.value for item in status.values}
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "state reads are failing before first sample"
    assert values["state_errors"] == "1"
    assert values["last_error"] == "CAN read timeout"
