import math

import pytest

from linkerbot_hand import SensorType, get_model_capabilities

from linkerbot_hand_ros.configuration import (
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
