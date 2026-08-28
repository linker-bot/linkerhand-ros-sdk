"""Validate lifecycle-aware ROS parameter updates without device I/O."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from rcl_interfaces.msg import SetParametersResult
from rclpy.lifecycle import LifecycleNode
from rclpy.parameter import Parameter

from .driver_config import (
    DRIVER_PARAMETERS,
    RUNTIME_RATE_PARAMETERS,
    STATIC_PARAMETERS,
    DriverConfiguration,
    read_driver_configuration,
)

PARAMETER_DEFAULTS = {
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


@dataclass(frozen=True, slots=True)
class ParameterUpdate:
    """Validation result and optional configured-state rate replacement."""

    result: SetParametersResult
    configuration: DriverConfiguration | None = None


def declare_driver_parameters(node: LifecycleNode) -> None:
    """Declare the complete HandDevice 1.0 parameter surface."""

    for name, value in PARAMETER_DEFAULTS.items():
        node.declare_parameter(name, value)


def prepare_parameter_update(
    parameters: list[Parameter],
    configuration: DriverConfiguration | None,
    *,
    read_parameter: Callable[[str], object],
    error_text: Callable[[BaseException], str],
) -> ParameterUpdate:
    """Validate a proposed atomic update without applying runtime effects."""

    proposed = {
        parameter.name: parameter.value
        for parameter in parameters
        if parameter.name in DRIVER_PARAMETERS
    }
    static_updates = set(proposed).intersection(STATIC_PARAMETERS)
    if configuration is not None and static_updates:
        names = ", ".join(sorted(static_updates))
        return ParameterUpdate(
            SetParametersResult(
                successful=False,
                reason=f"static parameters require cleanup before update: {names}",
            )
        )
    if configuration is None and proposed:
        try:
            read_driver_configuration(
                lambda name: proposed.get(name, read_parameter(name))
            )
        except Exception as error:  # noqa: BLE001 - parameter boundary
            return ParameterUpdate(
                SetParametersResult(successful=False, reason=error_text(error))
            )
        return ParameterUpdate(SetParametersResult(successful=True))

    updates = {
        name: value
        for name, value in proposed.items()
        if name in RUNTIME_RATE_PARAMETERS
    }
    if not updates:
        return ParameterUpdate(SetParametersResult(successful=True))
    if configuration is None:
        return ParameterUpdate(
            SetParametersResult(
                successful=False,
                reason="driver configuration is unavailable",
            )
        )
    try:
        updated = configuration.with_runtime_rates(updates)
    except Exception as error:  # noqa: BLE001 - parameter boundary
        return ParameterUpdate(
            SetParametersResult(successful=False, reason=error_text(error))
        )
    return ParameterUpdate(SetParametersResult(successful=True), updated)


__all__ = [
    "PARAMETER_DEFAULTS",
    "ParameterUpdate",
    "declare_driver_parameters",
    "prepare_parameter_update",
]
