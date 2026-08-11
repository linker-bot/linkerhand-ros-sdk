"""Validated driver configuration derived from ROS parameter values."""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from linkerbot_hand import (
    DeviceCapabilities,
    PollingPlan,
    SensorType,
    SessionConfig,
    TransportType,
    get_model_capabilities,
)


SENSOR_RATE_PARAMETERS = {
    SensorType.ANGLES: "angles_hz",
    SensorType.FORCE: "force_hz",
    SensorType.TEMPERATURE: "temperature_hz",
    SensorType.CURRENT: "current_hz",
    SensorType.FAULTS: "faults_hz",
}
RESERVED_TRANSPORT_OPTIONS = {"model", "side", "session_config", "clock"}


ParameterReader = Callable[[str], object]


@dataclass(frozen=True, slots=True)
class DriverConfiguration:
    """One validated configuration snapshot captured during configure."""

    model: str
    side: str
    transport_options: dict[str, Any]
    session_config: SessionConfig
    publish_rate_hz: float
    diagnostic_rate_hz: float
    sensor_rates: tuple[tuple[SensorType, float], ...]

    def polling_plan(self, capabilities: DeviceCapabilities) -> PollingPlan:
        """Build a capability-limited SDK polling plan."""
        rates: dict[str, float] = {}
        for sensor, requested in self.sensor_rates:
            sensor_capability = capabilities.sensor(sensor)
            parameter_name = SENSOR_RATE_PARAMETERS[sensor]
            rates[parameter_name] = (
                0.0
                if sensor_capability is None
                else min(requested, sensor_capability.recommended_hz)
            )
        return PollingPlan(**rates)


def read_driver_configuration(read_parameter: ParameterReader) -> DriverConfiguration:
    """Read and validate all node parameters without performing device I/O."""
    model = str(read_parameter("model"))
    side = str(read_parameter("side"))
    command_rate_hz = _finite_rate(
        "command_rate_hz", read_parameter("command_rate_hz"), allow_zero=False
    )
    publish_rate_hz = _finite_rate(
        "publish_rate_hz", read_parameter("publish_rate_hz"), allow_zero=False
    )
    diagnostic_rate_hz = _finite_rate(
        "diagnostic_rate", read_parameter("diagnostic_rate"), allow_zero=False
    )
    sensor_rates = tuple(
        (
            sensor,
            _finite_rate(
                parameter_name,
                read_parameter(parameter_name),
                allow_zero=True,
            ),
        )
        for sensor, parameter_name in SENSOR_RATE_PARAMETERS.items()
    )

    auto_reconnect = read_parameter("auto_reconnect")
    if not isinstance(auto_reconnect, bool):
        raise ValueError("auto_reconnect must be a boolean")

    transport_options = parse_transport_options(
        str(read_parameter("transport_options_json"))
    )
    if not transport_options:
        transport_options = legacy_transport_options(
            model=model,
            interface_name=str(read_parameter("interface_name")),
            interface_type=str(read_parameter("interface_type")),
            library_path=str(read_parameter("library_path")),
        )

    return DriverConfiguration(
        model=model,
        side=side,
        transport_options=transport_options,
        session_config=SessionConfig(
            command_rate_hz=command_rate_hz,
            auto_reconnect=auto_reconnect,
        ),
        publish_rate_hz=publish_rate_hz,
        diagnostic_rate_hz=diagnostic_rate_hz,
        sensor_rates=sensor_rates,
    )


def parse_transport_options(raw: str) -> dict[str, Any]:
    """Parse SDK transport keyword arguments from a JSON object."""
    try:
        options = json.loads(raw.strip() or "{}")
    except json.JSONDecodeError as error:
        raise ValueError(
            f"transport_options_json is invalid JSON: {error.msg}"
        ) from error
    if not isinstance(options, dict):
        raise ValueError("transport_options_json must contain a JSON object")
    reserved = RESERVED_TRANSPORT_OPTIONS.intersection(options)
    if reserved:
        names = ", ".join(sorted(reserved))
        raise ValueError(f"transport_options_json contains node-owned keys: {names}")
    return options


def legacy_transport_options(
    *,
    model: str,
    interface_name: str,
    interface_type: str,
    library_path: str,
) -> dict[str, Any]:
    """Translate deprecated transport parameters into SDK keyword arguments."""
    capabilities = get_model_capabilities(model)
    interface_name = interface_name.strip()
    interface_type = interface_type.strip()
    library_path = library_path.strip()
    options: dict[str, Any] = {}
    if interface_type:
        options["interface_type"] = interface_type
    if library_path:
        options["library_path"] = library_path
    if not interface_name or interface_type != "socketcan":
        return options
    if capabilities.transport is TransportType.CAN:
        options["interface_name"] = interface_name
    elif capabilities.model == "L30":
        options["channel"] = interface_name
    else:
        options["socketcan_channel"] = interface_name
    return options


def _finite_rate(name: str, value: object, *, allow_zero: bool) -> float:
    if isinstance(value, bool):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")
    try:
        rate = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}") from error
    invalid = not math.isfinite(rate) or rate < 0.0 or (not allow_zero and rate == 0.0)
    if invalid:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return rate


__all__ = [
    "DriverConfiguration",
    "SENSOR_RATE_PARAMETERS",
    "legacy_transport_options",
    "parse_transport_options",
    "read_driver_configuration",
]
