from dataclasses import replace

from diagnostic_msgs.msg import DiagnosticStatus

from linkerbot_hand import HandDevice, HandSide, PollingPlan, SensorType
from linkerbot_hand.models import get_model_capabilities
from linkerbot_hand.testing import FakeBackendFactory, wait_until

from linkerbot_hand_ros.diagnostics import build_diagnostic_status
from linkerbot_hand_ros.metrics import DriverMetrics


def test_repeated_sdk_errors_take_priority_over_stale_cache_warning():
    capabilities = get_model_capabilities("L6", side=HandSide.LEFT)
    backend_factory = FakeBackendFactory(capabilities)
    backend_factory.state.set_reading(SensorType.ANGLES, (100.0,) * 6)
    device = HandDevice(capabilities, backend_factory)
    device.open()
    try:
        device.start_polling(PollingPlan(angles_hz=60.0))
        wait_until(lambda: len(backend_factory.state.poll_log) >= 1)
        device.stop_polling()
        snapshot = device.get_snapshot()
        stale_snapshot = replace(
            snapshot,
            samples=tuple(
                replace(sample, stale=True)
                if sample.sensor is SensorType.ANGLES
                else sample
                for sample in snapshot.samples
            ),
        )
        health = replace(device.get_health(), consecutive_errors=3)

        status = build_diagnostic_status(
            node_name="/linkerbot_hand",
            active=True,
            capabilities=device.get_capabilities(),
            health=health,
            snapshot=stale_snapshot,
            metrics=DriverMetrics().snapshot(),
            latest_handles={},
        )

        assert status.level == DiagnosticStatus.ERROR
        assert status.message == "repeated SDK I/O errors"
    finally:
        device.close()


def test_ros_boundary_failure_is_reported_as_warning():
    capabilities = get_model_capabilities("L6", side=HandSide.LEFT)
    backend_factory = FakeBackendFactory(capabilities)
    backend_factory.state.set_reading(SensorType.ANGLES, (100.0,) * 6)
    device = HandDevice(capabilities, backend_factory)
    device.open()
    try:
        device.start_polling(PollingPlan(angles_hz=60.0))
        wait_until(lambda: len(backend_factory.state.poll_log) >= 1)
        device.stop_polling()
        metrics = DriverMetrics()
        metrics.record_command_error(ValueError("invalid target"))

        status = build_diagnostic_status(
            node_name="/linkerbot_hand",
            active=True,
            capabilities=device.get_capabilities(),
            health=device.get_health(),
            snapshot=device.get_snapshot(),
            metrics=metrics.snapshot(),
            latest_handles={},
        )

        assert status.level == DiagnosticStatus.WARN
        assert status.message == "latest ROS driver operation failed"
        values = {item.key: item.value for item in status.values}
        assert values["ros_recent_error"] == "invalid target"
    finally:
        device.close()
