import pytest
from linkerbot_hand import (
    CommandStage,
    HandDevice,
    HandSide,
    PollingPlan,
    SensorType,
    ValidationError,
    get_model_capabilities,
)
from linkerbot_hand.testing import FakeBackendFactory, wait_until

from linkerbot_hand_ros.adapter import (
    HandAdapter,
    normalize_command,
    normalize_model,
    validate_command,
)

MODELS = ("o6", "l6", "l20lite", "l25", "o20", "o30i", "l30")


@pytest.fixture
def adapter_rig():
    capabilities = get_model_capabilities("L6", side=HandSide.LEFT)
    backend_factory = FakeBackendFactory(capabilities)
    device = HandDevice(capabilities, backend_factory)
    adapter = HandAdapter("l6", side="left", device=device)
    try:
        yield adapter, backend_factory
    finally:
        adapter.close()


def test_sdk_capabilities_are_the_only_model_metadata_source():
    assert {model: get_model_capabilities(model).joint_count for model in MODELS} == {
        "o6": 6,
        "l6": 6,
        "l20lite": 10,
        "l25": 16,
        "o20": 16,
        "o30i": 20,
        "l30": 17,
    }
    assert all(
        get_model_capabilities(model).percentage_convention == "100=open, 0=closed"
        for model in MODELS
    )
    assert normalize_model("L20-Lite") == "l20lite"


def test_adapter_construction_performs_no_device_io(adapter_rig):
    adapter, backend_factory = adapter_rig

    assert adapter.model == "l6"
    assert adapter.joint_names == get_model_capabilities("L6").joint_names
    assert backend_factory.state.open_count == 0
    assert backend_factory.state.io_count == 0


def test_adapter_uses_sdk_polling_and_cached_snapshot(adapter_rig):
    adapter, backend_factory = adapter_rig
    backend_factory.state.set_reading(
        SensorType.ANGLES,
        (100.0, None, 80.0, 70.0, 60.0, 50.0),
        raw_values=(255, 255, 204, 178, 153, 128),
    )

    adapter.open()
    adapter.start_polling(PollingPlan(angles_hz=60.0))
    wait_until(lambda: len(backend_factory.state.poll_log) >= 1)
    adapter.stop_polling()
    io_count = backend_factory.state.io_count

    snapshot = adapter.get_snapshot()
    health = adapter.get_health()

    assert snapshot.angles is not None
    assert snapshot.angles.values == (100.0, None, 80.0, 70.0, 60.0, 50.0)
    assert snapshot.angles.valid
    assert not snapshot.angles.stale
    assert health.connected
    assert backend_factory.state.io_count == io_count


def test_continuous_commands_return_handles_without_waiting(adapter_rig):
    adapter, backend_factory = adapter_rig
    adapter.open()

    handle = adapter.submit_angles([100, 50, 80, 70, 60, 50])

    assert handle.command.values == (100.0, 50.0, 80.0, 70.0, 60.0, 50.0)
    wait_until(lambda: handle.done)
    assert handle.stage is CommandStage.DISPATCHED
    assert backend_factory.state.dispatched_commands[-1] == handle.command


def test_raw_angles_use_exact_protocol_coordinates(adapter_rig):
    adapter, backend_factory = adapter_rig
    adapter.open()

    handle = adapter.submit_raw_angles([255, 128, 204, 178, 153, 0])

    assert handle.command.values == (255, 128, 204, 178, 153, 0)
    wait_until(lambda: handle.done)
    assert handle.stage is CommandStage.DISPATCHED
    assert backend_factory.state.dispatched_commands[-1] == handle.command


def test_raw_angle_validation_uses_joint_protocol_ranges(adapter_rig):
    adapter, backend_factory = adapter_rig
    adapter.open()

    with pytest.raises(ValidationError, match=r"\[0, 255\]"):
        adapter.submit_raw_angles([256] * 6)
    with pytest.raises(ValidationError, match="must be an integer"):
        adapter.submit_raw_angles([10.5] * 6)

    assert backend_factory.state.dispatched_commands == []


@pytest.mark.parametrize(
    "values",
    ([1] * 5, [1] * 7, [-1] * 6, [101] * 6, [float("nan")] * 6),
)
def test_invalid_angle_commands_are_rejected_before_sdk_submission(adapter_rig, values):
    adapter, backend_factory = adapter_rig
    adapter.open()

    with pytest.raises(ValidationError):
        adapter.submit_angles(values)

    assert backend_factory.state.dispatched_commands == []


def test_ros_float_arrays_are_normalized_for_integer_only_models():
    capabilities = get_model_capabilities("L30", side=HandSide.RIGHT)
    backend_factory = FakeBackendFactory(capabilities)
    adapter = HandAdapter(
        "l30",
        side="right",
        device=HandDevice(capabilities, backend_factory),
    )
    adapter.open()
    try:
        speed = adapter.submit_speeds([250.0] * 17)
        torque = adapter.submit_torques([800.0] * 17)
        wait_until(lambda: speed.done and torque.done)

        assert speed.command.values == (250,) * 17
        assert torque.command.values == (800,) * 17
        with pytest.raises(ValidationError, match="must be an integer"):
            adapter.submit_speeds([10.5] * 17)
    finally:
        adapter.close()


def test_validation_uses_each_joint_range_from_sdk_capabilities():
    assert validate_command("l6", [100, 50, 100, 100, 100, 100], "angles") == [
        100.0,
        50.0,
        100.0,
        100.0,
        100.0,
        100.0,
    ]
    with pytest.raises(ValidationError, match=r"\[60, 800\]"):
        validate_command("l30", [59.0] * 17, "torques")
    with pytest.raises(ValidationError, match="unsupported command"):
        normalize_command(get_model_capabilities("l6"), [1] * 6, "position")


def test_close_is_idempotent(adapter_rig):
    adapter, backend_factory = adapter_rig
    adapter.open()

    adapter.close()
    adapter.close()

    assert backend_factory.state.open_count == 1
    assert backend_factory.state.close_count == 1
    assert backend_factory.state.active_connections == 0
