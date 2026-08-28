import math

import pytest
from linkerbot_hand import (
    HandSide,
    UnsupportedOperationError,
    ValidationError,
)
from linkerbot_hand_ros.driver_config import (
    DriverConfiguration,
    read_driver_configuration,
)
from linkerbot_hand_ros.transport_config import (
    PythonCANConfiguration,
    SocketCANConfiguration,
    VendorCANFDConfiguration,
    parse_transport_configuration,
)


def _parameters(**overrides):
    values = {
        "model": "l6",
        "side": "left",
        "transport_json": '{"backend":"socketcan","channel":"can0"}',
        "command_rate_hz": 250.0,
        "discrete_queue_capacity": 16,
        "mailbox_capacity": 256,
        "route_queue_capacity": 256,
        "identity_timeout_s": 0.2,
        "transaction_timeout_s": 0.1,
        "shutdown_timeout_s": 2.0,
        "auto_reconnect": False,
        "reconnect_initial_delay_s": 0.1,
        "reconnect_max_delay_s": 2.0,
        "publish_rate_hz": 100.0,
        "diagnostic_rate_hz": 1.0,
        "angles_hz": 100.0,
        "speeds_hz": 0.0,
        "force_hz": 0.0,
        "temperature_hz": 0.0,
        "current_hz": 0.0,
        "faults_hz": 0.0,
        "torques_hz": 0.0,
        "voltage_hz": 0.0,
    }
    values.update(overrides)
    return values


def test_configuration_uses_declared_sdk_capabilities_as_source_of_truth():
    configuration = read_driver_configuration(
        _parameters(model="O6", side="right", force_hz=30.0).__getitem__
    )

    assert isinstance(configuration, DriverConfiguration)
    assert configuration.model == "O6"
    assert configuration.side == "right"
    assert configuration.declared_capabilities.side is HandSide.RIGHT
    assert configuration.declared_capabilities.identity.verified is False
    assert configuration.polling_plan.angles_hz == 100.0
    assert configuration.polling_plan.force_hz == 30.0


def test_l30_requires_an_empty_side_parameter():
    configuration = read_driver_configuration(
        _parameters(model="l30", side="").__getitem__
    )

    assert configuration.model == "L30"
    assert configuration.side is None
    assert configuration.declared_capabilities.side is HandSide.NOT_APPLICABLE

    with pytest.raises(ValidationError, match="L30 must not specify side"):
        read_driver_configuration(_parameters(model="l30", side="left").__getitem__)


def test_configuration_rejects_polling_not_supported_by_capability():
    with pytest.raises(UnsupportedOperationError, match="cannot poll measured_speeds"):
        read_driver_configuration(_parameters(speeds_hz=1.0).__getitem__)


def test_runtime_rate_update_is_atomic_and_capability_checked():
    configuration = read_driver_configuration(_parameters().__getitem__)
    updated = configuration.with_runtime_rates(
        {
            "angles_hz": 60.0,
            "current_hz": 10.0,
            "publish_rate_hz": 120.0,
            "diagnostic_rate_hz": 2.0,
        }
    )

    assert configuration.polling_plan.angles_hz == 100.0
    assert updated.polling_plan.angles_hz == 60.0
    assert updated.polling_plan.current_hz == 10.0
    assert updated.publish_rate_hz == 120.0
    assert updated.diagnostic_rate_hz == 2.0

    with pytest.raises(UnsupportedOperationError):
        configuration.with_runtime_rates({"speeds_hz": 1.0})
    with pytest.raises(ValidationError, match="not runtime-adjustable"):
        configuration.with_runtime_rates({"model": "o6"})


@pytest.mark.parametrize(
    "raw, expected",
    [
        (
            ('{"backend":"socketcan","channel":"can3","nominal_bitrate":1000000}'),
            SocketCANConfiguration(channel="can3", nominal_bitrate=1_000_000),
        ),
        (
            (
                '{"backend":"python_can","interface_name":"PCAN_USBBUS1",'
                '"interface_type":"pcan"}'
            ),
            PythonCANConfiguration("PCAN_USBBUS1", "pcan"),
        ),
        (
            '{"backend":"vendor_canfd","device_index":1,"channel_index":2}',
            VendorCANFDConfiguration(device_index=1, channel_index=2),
        ),
    ],
)
def test_transport_json_parses_to_typed_configuration(raw, expected):
    assert parse_transport_configuration(raw) == expected


@pytest.mark.parametrize(
    "raw, error",
    [
        ("[]", "JSON object"),
        ("{}", "backend must be one of"),
        ('{"backend":"serial"}', "backend must be one of"),
        ('{"backend":"socketcan"}', "requires channel"),
        (
            '{"backend":"socketcan","channel":"can0","model":"l6"}',
            "unknown keys: model",
        ),
    ],
)
def test_transport_json_rejects_ambiguous_or_unowned_fields(raw, error):
    with pytest.raises(ValidationError, match=error):
        parse_transport_configuration(raw)


@pytest.mark.parametrize(
    "name, value",
    [
        ("angles_hz", math.nan),
        ("publish_rate_hz", math.inf),
        ("diagnostic_rate_hz", True),
    ],
)
def test_configuration_rejects_invalid_rates(name, value):
    with pytest.raises(ValidationError, match=f"{name} must be finite"):
        read_driver_configuration(_parameters(**{name: value}).__getitem__)


def test_configuration_delegates_runtime_limits_to_sdk_device_config():
    with pytest.raises(ValidationError, match="mailbox_capacity"):
        read_driver_configuration(_parameters(mailbox_capacity=0).__getitem__)

    with pytest.raises(ValidationError, match="auto_reconnect"):
        read_driver_configuration(_parameters(auto_reconnect="false").__getitem__)
