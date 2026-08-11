"""ROS-facing adapter for the transport-neutral Linkerbot hand SDK."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from linkerbot_hand import (
    CommandHandle,
    CommandPolicy,
    DeviceCapabilities,
    HandDevice,
    HandSnapshot,
    HealthStatus,
    NumericRange,
    PollingPlan,
    SessionConfig,
    UnsupportedOperationError,
    ValidationError,
    get_model_capabilities,
)
from linkerbot_hand.backends.legacy import create_hand_device


def normalize_model(model: str) -> str:
    """Return the SDK canonical model name in the historical ROS casing."""
    return get_model_capabilities(model).model.lower()


def _ranges_for_command(
    capabilities: DeviceCapabilities, command: str
) -> tuple[NumericRange, ...]:
    if command == "angles":
        if any(joint.percentage_mapping is None for joint in capabilities.joints):
            raise UnsupportedOperationError(
                "percentage angle control is not supported for this hand side"
            )
        return tuple(NumericRange(0, 100, "percent") for _ in capabilities.joints)
    if command == "speeds":
        ranges = tuple(joint.speed_range for joint in capabilities.joints)
    elif command == "torques":
        ranges = tuple(joint.torque_range for joint in capabilities.joints)
    else:
        raise ValidationError(f"unsupported command: {command}")
    if any(bounds is None for bounds in ranges):
        raise UnsupportedOperationError(
            f"{command.removesuffix('s')} control is not supported"
        )
    return tuple(bounds for bounds in ranges if bounds is not None)


def normalize_command(
    capabilities: DeviceCapabilities,
    values: Sequence[float],
    command: str,
) -> list[float | int]:
    """Normalize a ROS float array using SDK-provided joint capabilities."""
    if isinstance(values, (str, bytes)):
        raise ValidationError("command values must be a numeric sequence")
    ranges = _ranges_for_command(capabilities, command)
    if len(values) != capabilities.joint_count:
        raise ValidationError(
            f"{command} requires {capabilities.joint_count} values, got {len(values)}"
        )

    normalized: list[float | int] = []
    for index, (value, bounds) in enumerate(zip(values, ranges)):
        numeric = _finite_number(value, command, index)
        if bounds.integer_only and not numeric.is_integer():
            raise ValidationError(f"{command}[{index}] must be an integer")
        _validate_range(numeric, bounds, command, index)
        normalized.append(int(numeric) if bounds.integer_only else numeric)
    return normalized


def _finite_number(value: object, command: str, index: int) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValidationError(f"{command}[{index}] must be numeric")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValidationError(f"{command}[{index}] must be finite")
    return numeric


def _validate_range(
    value: float,
    bounds: NumericRange,
    command: str,
    index: int,
) -> None:
    if not bounds.minimum <= value <= bounds.maximum:
        raise ValidationError(
            f"{command}[{index}] must be in [{bounds.minimum}, {bounds.maximum}]"
        )
    if bounds.step is not None:
        steps = (value - bounds.minimum) / bounds.step
        if not math.isclose(steps, round(steps), abs_tol=1e-9):
            raise ValidationError(f"{command}[{index}] must use step {bounds.step}")


def validate_command(
    model: str,
    values: Sequence[float],
    command: str,
) -> list[float | int]:
    """Compatibility helper used by persistent ROS command clients."""
    return normalize_command(get_model_capabilities(model), values, command)


DeviceFactory = Callable[..., HandDevice]


class HandAdapter:
    """Small lifecycle and message boundary around one ``HandDevice``."""

    def __init__(
        self,
        model: str,
        *,
        side: str = "left",
        transport_options: Mapping[str, Any] | None = None,
        session_config: SessionConfig | None = None,
        device_factory: DeviceFactory = create_hand_device,
        device: HandDevice | None = None,
    ) -> None:
        if device is not None and transport_options:
            raise ValueError("transport_options cannot be used with an injected device")
        self._device = device or device_factory(
            model,
            side=side,
            session_config=session_config,
            **dict(transport_options or {}),
        )
        capabilities = self._device.get_capabilities()
        requested = get_model_capabilities(model, side=side)
        if capabilities.model != requested.model:
            raise ValidationError(
                f"injected device model {capabilities.model} does not match "
                f"requested model {requested.model}"
            )

    @property
    def model(self) -> str:
        return self.capabilities.model.lower()

    @property
    def capabilities(self) -> DeviceCapabilities:
        return self._device.get_capabilities()

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.capabilities.joint_names

    def open(self) -> None:
        self._device.open()

    def start_polling(self, plan: PollingPlan) -> None:
        self._device.start_polling(plan)

    def stop_polling(self) -> None:
        self._device.stop_polling()

    def submit_angles(self, values: Sequence[float]) -> CommandHandle:
        normalized = normalize_command(self.capabilities, values, "angles")
        return self._device.submit_angles(normalized, policy=CommandPolicy.LATEST)

    def submit_speeds(self, values: Sequence[float]) -> CommandHandle:
        normalized = normalize_command(self.capabilities, values, "speeds")
        return self._device.submit_speeds(normalized, policy=CommandPolicy.LATEST)

    def submit_torques(self, values: Sequence[float]) -> CommandHandle:
        normalized = normalize_command(self.capabilities, values, "torques")
        return self._device.submit_torques(normalized, policy=CommandPolicy.LATEST)

    def clear_faults(self) -> CommandHandle:
        return self._device.clear_faults()

    def get_snapshot(self) -> HandSnapshot:
        return self._device.get_snapshot()

    def get_health(self) -> HealthStatus:
        return self._device.get_health()

    def close(self) -> None:
        self._device.close()
