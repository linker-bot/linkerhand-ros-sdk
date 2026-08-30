"""Validated, zero-I/O configuration for the Hand SDK 1.0 ROS driver."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, fields, replace

from linkerbot_hand import (
    AngleFeedbackPolicy,
    DeviceCapabilities,
    DeviceConfig,
    HandSide,
    PollingPlan,
    TelemetrySemantic,
    UnsupportedOperationError,
    ValidationError,
    get_declared_capabilities,
)

from .transport_config import TransportConfiguration, parse_transport_configuration

POLL_RATE_PARAMETERS: Mapping[str, TelemetrySemantic] = {
    "angles_hz": TelemetrySemantic.ANGLES,
    "speeds_hz": TelemetrySemantic.MEASURED_SPEEDS,
    "force_hz": TelemetrySemantic.FORCE,
    "temperature_hz": TelemetrySemantic.TEMPERATURES,
    "current_hz": TelemetrySemantic.CURRENTS,
    "faults_hz": TelemetrySemantic.FAULTS,
    "torques_hz": TelemetrySemantic.TORQUES,
    "voltage_hz": TelemetrySemantic.VOLTAGES,
}
DEVICE_CONFIG_PARAMETERS = frozenset(
    {
        "command_rate_hz",
        "discrete_queue_capacity",
        "mailbox_capacity",
        "route_queue_capacity",
        "identity_timeout_s",
        "transaction_timeout_s",
        "shutdown_timeout_s",
        "auto_reconnect",
        "reconnect_initial_delay_s",
        "reconnect_max_delay_s",
    }
)
RUNTIME_RATE_PARAMETERS = frozenset(
    (
        *POLL_RATE_PARAMETERS,
        "angle_feedback",
        "publish_rate_hz",
        "diagnostic_rate_hz",
    )
)
STATIC_PARAMETERS = frozenset(
    {"model", "side", "transport_json", *DEVICE_CONFIG_PARAMETERS}
)
DRIVER_PARAMETERS = frozenset((*STATIC_PARAMETERS, *RUNTIME_RATE_PARAMETERS))

ParameterReader = Callable[[str], object]


@dataclass(frozen=True, slots=True)
class DriverConfiguration:
    """One immutable parameter snapshot captured during configure."""

    declared_capabilities: DeviceCapabilities
    transport: TransportConfiguration
    device_config: DeviceConfig
    polling_plan: PollingPlan
    publish_rate_hz: float
    diagnostic_rate_hz: float

    @property
    def model(self) -> str:
        return self.declared_capabilities.model

    @property
    def side(self) -> str | None:
        side = self.declared_capabilities.side
        return None if side is HandSide.NOT_APPLICABLE else side.value

    def with_runtime_rates(
        self,
        updates: Mapping[str, object],
    ) -> DriverConfiguration:
        """Return a validated copy containing only rate parameter changes."""

        unknown = set(updates).difference(RUNTIME_RATE_PARAMETERS)
        if unknown:
            names = ", ".join(sorted(unknown))
            raise ValidationError(
                f"parameters are not runtime-adjustable rates: {names}"
            )

        plan_values = {
            item.name: getattr(self.polling_plan, item.name)
            for item in fields(PollingPlan)
        }
        for name in POLL_RATE_PARAMETERS:
            if name in updates:
                plan_values[name] = _finite_rate(name, updates[name], allow_zero=True)
        if "angle_feedback" in updates:
            plan_values["angle_feedback"] = _angle_feedback_policy(
                updates["angle_feedback"]
            )
        plan = PollingPlan(**plan_values)
        validate_polling_plan(self.declared_capabilities, plan)

        publish_rate_hz = self.publish_rate_hz
        if "publish_rate_hz" in updates:
            publish_rate_hz = _finite_rate(
                "publish_rate_hz", updates["publish_rate_hz"], allow_zero=True
            )
        diagnostic_rate_hz = self.diagnostic_rate_hz
        if "diagnostic_rate_hz" in updates:
            diagnostic_rate_hz = _finite_rate(
                "diagnostic_rate_hz",
                updates["diagnostic_rate_hz"],
                allow_zero=True,
            )
        return replace(
            self,
            polling_plan=plan,
            publish_rate_hz=publish_rate_hz,
            diagnostic_rate_hz=diagnostic_rate_hz,
        )


def read_driver_configuration(read_parameter: ParameterReader) -> DriverConfiguration:
    """Read all final driver parameters without opening or querying a device."""

    model_value = read_parameter("model")
    if not isinstance(model_value, str) or not model_value.strip():
        raise ValidationError("model must be a non-empty string")
    model = model_value.strip()
    side_value = read_parameter("side")
    if not isinstance(side_value, str):
        raise ValidationError("side must be a string")
    side = side_value.strip() or None
    capabilities = get_declared_capabilities(model, side=side)

    transport = parse_transport_configuration(read_parameter("transport_json"))
    device_config = DeviceConfig(
        command_rate_hz=read_parameter("command_rate_hz"),
        discrete_queue_capacity=read_parameter("discrete_queue_capacity"),
        mailbox_capacity=read_parameter("mailbox_capacity"),
        route_queue_capacity=read_parameter("route_queue_capacity"),
        identity_timeout_s=read_parameter("identity_timeout_s"),
        transaction_timeout_s=read_parameter("transaction_timeout_s"),
        shutdown_timeout_s=read_parameter("shutdown_timeout_s"),
        auto_reconnect=read_parameter("auto_reconnect"),
        reconnect_initial_delay_s=read_parameter("reconnect_initial_delay_s"),
        reconnect_max_delay_s=read_parameter("reconnect_max_delay_s"),
    )
    plan = PollingPlan(
        **{
            name: _finite_rate(name, read_parameter(name), allow_zero=True)
            for name in POLL_RATE_PARAMETERS
        },
        angle_feedback=_angle_feedback_policy(read_parameter("angle_feedback")),
    )
    validate_polling_plan(capabilities, plan)
    return DriverConfiguration(
        declared_capabilities=capabilities,
        transport=transport,
        device_config=device_config,
        polling_plan=plan,
        publish_rate_hz=_finite_rate(
            "publish_rate_hz",
            read_parameter("publish_rate_hz"),
            allow_zero=True,
        ),
        diagnostic_rate_hz=_finite_rate(
            "diagnostic_rate_hz",
            read_parameter("diagnostic_rate_hz"),
            allow_zero=True,
        ),
    )


def validate_polling_plan(
    capabilities: DeviceCapabilities,
    plan: PollingPlan,
) -> None:
    """Reject requested query channels not declared by the selected profile."""

    if not isinstance(capabilities, DeviceCapabilities):
        raise ValidationError("capabilities must be DeviceCapabilities")
    if not isinstance(plan, PollingPlan):
        raise ValidationError("plan must be PollingPlan")
    for parameter_name, semantic in POLL_RATE_PARAMETERS.items():
        if getattr(plan, parameter_name) == 0:
            continue
        telemetry = capabilities.telemetry_channel(semantic)
        if telemetry is None or not telemetry.device_queried:
            raise UnsupportedOperationError(
                f"{capabilities.model} cannot poll {semantic.value}",
                operation="start_polling",
                model=capabilities.model,
            )


def _finite_rate(name: str, value: object, *, allow_zero: bool) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0
        or (not allow_zero and value == 0)
    ):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValidationError(f"{name} must be finite and {qualifier}")
    return float(value)


def _angle_feedback_policy(value: object) -> AngleFeedbackPolicy:
    if not isinstance(value, str) or not value.strip():
        raise ValidationError("angle_feedback must be a non-empty string")
    try:
        return AngleFeedbackPolicy(value.strip().lower())
    except ValueError as error:
        choices = ", ".join(policy.value for policy in AngleFeedbackPolicy)
        raise ValidationError(f"angle_feedback must be one of: {choices}") from error


__all__ = [
    "DEVICE_CONFIG_PARAMETERS",
    "DRIVER_PARAMETERS",
    "POLL_RATE_PARAMETERS",
    "RUNTIME_RATE_PARAMETERS",
    "STATIC_PARAMETERS",
    "DriverConfiguration",
    "read_driver_configuration",
    "validate_polling_plan",
]
