import math

import pytest

from linkerbot_hand import MODEL_CAPABILITIES, SensorType, get_model_capabilities

from linkerbot_hand_ros.configuration import (
    SENSOR_RATE_PARAMETERS,
    legacy_transport_options,
    parse_transport_options,
    read_driver_configuration,
)


def _parameters(**overrides):
    values = {
        "model": "l6",
        "side": "left",
        "transport_options_json": "{}",
        "interface_name": "can0",
        "interface_type": "socketcan",
        "library_path": "",
        "command_rate_hz": 100.0,
        "auto_reconnect": False,
        "publish_rate_hz": 60.0,
        "angles_hz": 60.0,
        "speeds_hz": 0.0,
        "torques_hz": 0.0,
        "force_hz": 0.0,
        "temperature_hz": 1.0,
        "current_hz": 10.0,
        "faults_hz": 5.0,
        "diagnostic_rate": 1.0,
    }
    values.update(overrides)
    return values


def test_configuration_is_snapshotted_and_json_options_take_precedence():
    parameters = _parameters(
        transport_options_json='{"interface_name":"can7","interface_type":"socketcan"}'
    )

    configuration = read_driver_configuration(parameters.__getitem__)
    parameters["angles_hz"] = 0.0

    assert configuration.transport_options == {
        "interface_name": "can7",
        "interface_type": "socketcan",
    }
    assert dict(configuration.sensor_rates)[SensorType.ANGLES] == 60.0


def test_polling_plan_caps_rates_and_disables_unsupported_sensors():
    configuration = read_driver_configuration(
        _parameters(
            model="o30i",
            side="unknown",
            angles_hz=999.0,
            speeds_hz=999.0,
            torques_hz=999.0,
            force_hz=999.0,
            temperature_hz=999.0,
            current_hz=999.0,
            faults_hz=999.0,
        ).__getitem__
    )
    capabilities = get_model_capabilities("o30i")

    plan = configuration.polling_plan(capabilities)

    assert plan.force_hz == 0.0
    assert plan.angles_hz == capabilities.sensor(SensorType.ANGLES).recommended_hz
    assert plan.speeds_hz == capabilities.sensor(SensorType.SPEEDS).recommended_hz
    assert plan.torques_hz == capabilities.sensor(SensorType.TORQUES).recommended_hz
    assert (
        plan.temperature_hz
        == capabilities.sensor(SensorType.TEMPERATURE).recommended_hz
    )


@pytest.mark.parametrize(
    ("model", "expected_key"),
    (("l6", "interface_name"), ("l30", "channel"), ("o20", "socketcan_channel")),
)
def test_legacy_socketcan_parameters_use_sdk_model_transport(model, expected_key):
    assert legacy_transport_options(
        model=model,
        interface_name="can3",
        interface_type="socketcan",
        library_path="",
    ) == {"interface_type": "socketcan", expected_key: "can3"}


def test_configuration_rejects_reserved_transport_keys_and_non_finite_rates():
    with pytest.raises(ValueError, match="node-owned keys: model"):
        parse_transport_options('{"model":"l6"}')

    with pytest.raises(ValueError, match="angles_hz must be finite"):
        read_driver_configuration(_parameters(angles_hz=math.nan).__getitem__)

    with pytest.raises(ValueError, match="command_rate_hz must be finite"):
        read_driver_configuration(_parameters(command_rate_hz=True).__getitem__)


def test_runtime_rates_can_be_updated_and_disabled():
    configuration = read_driver_configuration(_parameters().__getitem__)

    updated = configuration.with_runtime_rates(
        {
            "speeds_hz": 20.0,
            "torques_hz": 30.0,
            "angles_hz": 0.0,
            "publish_rate_hz": 0.0,
            "diagnostic_rate": 0.0,
        }
    )
    plan = updated.polling_plan(get_model_capabilities("l6"))

    assert plan.angles_hz == 0.0
    assert plan.speeds_hz == 0.0
    assert plan.torques_hz == 30.0
    assert updated.publish_rate_hz == 0.0
    assert updated.diagnostic_rate_hz == 0.0


def test_runtime_speed_rate_uses_sdk_capability_for_supported_model():
    configuration = read_driver_configuration(
        _parameters(model="o6", side="right").__getitem__
    ).with_runtime_rates({"speeds_hz": 20.0})

    plan = configuration.polling_plan(get_model_capabilities("o6", side="right"))

    assert plan.speeds_hz == 20.0


@pytest.mark.parametrize("model", MODEL_CAPABILITIES)
def test_polling_plan_uses_sdk_sensor_capabilities_for_every_model(model):
    configuration = read_driver_configuration(
        _parameters(
            model=model,
            angles_hz=999.0,
            speeds_hz=999.0,
            torques_hz=999.0,
            force_hz=999.0,
            temperature_hz=999.0,
            current_hz=999.0,
            faults_hz=999.0,
        ).__getitem__
    )
    capabilities = get_model_capabilities(model)

    rates = configuration.polling_plan(capabilities).rates()

    for sensor in SENSOR_RATE_PARAMETERS:
        sensor_capability = capabilities.sensor(sensor)
        expected = (
            0.0
            if sensor_capability is None
            else sensor_capability.recommended_hz
        )
        assert rates[sensor] == expected


@pytest.mark.parametrize("model", MODEL_CAPABILITIES)
def test_runtime_speed_and_torque_rates_can_be_disabled_for_every_model(model):
    capabilities = get_model_capabilities(model)
    configuration = read_driver_configuration(
        _parameters(model=model).__getitem__
    ).with_runtime_rates({"speeds_hz": 999.0, "torques_hz": 999.0})

    enabled = configuration.polling_plan(capabilities).rates()

    for sensor in (SensorType.SPEEDS, SensorType.TORQUES):
        sensor_capability = capabilities.sensor(sensor)
        expected = (
            0.0
            if sensor_capability is None
            else sensor_capability.recommended_hz
        )
        assert enabled[sensor] == expected

    disabled = configuration.with_runtime_rates(
        {"speeds_hz": 0.0, "torques_hz": 0.0}
    ).polling_plan(capabilities)

    assert disabled.speeds_hz == 0.0
    assert disabled.torques_hz == 0.0
