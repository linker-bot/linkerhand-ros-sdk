"""Hardware-independent adapter around the Linkerbot hand SDK."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Any


@dataclass(frozen=True)
class ModelSpec:
    sdk_class: str
    joint_names: tuple[str, ...]
    transport: str
    speed_range: tuple[int, int] = (0, 100)
    torque_range: tuple[int, int] = (0, 100)


SIX_JOINT_NAMES = (
    "thumb_flex",
    "thumb_abd",
    "index",
    "middle",
    "ring",
    "pinky",
)

MODEL_SPECS: dict[str, ModelSpec] = {
    "o6": ModelSpec("O6", SIX_JOINT_NAMES, "can"),
    "l6": ModelSpec("L6", SIX_JOINT_NAMES, "can"),
    "l20lite": ModelSpec(
        "L20lite",
        (
            "thumb_flex",
            "thumb_abd",
            "index_flex",
            "middle_flex",
            "ring_flex",
            "pinky_flex",
            "index_abd",
            "ring_abd",
            "pinky_abd",
            "thumb_yaw",
        ),
        "can",
    ),
    "l25": ModelSpec(
        "L25",
        (
            "thumb_abd",
            "thumb_yaw",
            "thumb_root1",
            "thumb_tip",
            "index_abd",
            "index_root1",
            "index_tip",
            "middle_abd",
            "middle_root1",
            "middle_tip",
            "ring_abd",
            "ring_root1",
            "ring_tip",
            "pinky_abd",
            "pinky_root1",
            "pinky_tip",
        ),
        "can",
    ),
    "o20": ModelSpec(
        "O20",
        (
            "thumb_mcp",
            "thumb_ip",
            "thumb_abd",
            "thumb_cmc",
            "index_abd",
            "index_mcp",
            "index_pip",
            "middle_abd",
            "middle_mcp",
            "middle_pip",
            "ring_abd",
            "ring_mcp",
            "ring_pip",
            "pinky_abd",
            "pinky_mcp",
            "pinky_dip",
        ),
        "canfd",
        torque_range=(0, 1000),
    ),
    "o30i": ModelSpec(
        "O30i",
        (
            "roll_0",
            "yaw_0",
            "yaw_1",
            "yaw_2",
            "yaw_3",
            "yaw_4",
            "root1_0",
            "root1_1",
            "root1_2",
            "root1_3",
            "root1_4",
            "root2_1",
            "root2_2",
            "root2_3",
            "root2_4",
            "tip_0",
            "tip_1",
            "tip_2",
            "tip_3",
            "tip_4",
        ),
        "canfd",
        speed_range=(0, 255),
        torque_range=(0, 255),
    ),
    "l30": ModelSpec(
        "L30",
        tuple(f"j{index}" for index in range(1, 18)),
        "canfd",
        speed_range=(1, 250),
        torque_range=(60, 800),
    ),
}


def normalize_model(model: str) -> str:
    normalized = model.strip().lower().replace("-", "")
    aliases = {"l20lite": "l20lite", "o30i": "o30i"}
    normalized = aliases.get(normalized, normalized)
    if normalized not in MODEL_SPECS:
        supported = ", ".join(sorted(MODEL_SPECS))
        raise ValueError(f"unsupported model {model!r}; choose one of: {supported}")
    return normalized


def create_sdk_hand(
    model: str,
    *,
    side: str,
    interface_name: str,
    interface_type: str,
    library_path: str | None = None,
    sdk_module: Any | None = None,
) -> Any:
    """Construct the appropriate SDK hand using its transport-specific API."""
    model = normalize_model(model)
    spec = MODEL_SPECS[model]
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")

    if sdk_module is None:
        import linkerbot.hand as sdk_module

    hand_class = getattr(sdk_module, spec.sdk_class)
    if spec.transport == "can":
        return hand_class(
            side=side,
            interface_name=interface_name,
            interface_type=interface_type,
        )

    common: dict[str, Any] = {"interface_type": interface_type}
    if library_path:
        common["library_path"] = library_path
    if model == "l30":
        if interface_type == "socketcan":
            common["channel"] = interface_name
        return hand_class(**common)
    if interface_type == "socketcan":
        common["socketcan_channel"] = interface_name
    if model == "o20":
        common["side"] = side
    return hand_class(**common)


def _as_float_list(value: Any) -> list[float]:
    if hasattr(value, "to_list"):
        value = value.to_list()
    elif hasattr(value, "values"):
        value = value.values
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise TypeError(f"SDK returned unsupported vector type {type(value).__name__}")
    return [float(item.value if isinstance(item, Enum) else item) for item in value]


def validate_command(
    model: str,
    values: Sequence[float],
    command: str,
) -> list[float] | list[int]:
    """Validate and normalize one model-specific SDK command."""
    model = normalize_model(model)
    spec = MODEL_SPECS[model]
    limits_by_command = {
        "angles": (0, 100),
        "speeds": spec.speed_range,
        "torques": spec.torque_range,
    }
    if command not in limits_by_command:
        raise ValueError(f"unsupported command: {command}")

    result = [float(value) for value in values]
    if len(result) != len(spec.joint_names):
        raise ValueError(
            f"{command} requires {len(spec.joint_names)} values, got {len(result)}"
        )
    if any(not math.isfinite(value) for value in result):
        raise ValueError(f"{command} values must be finite")
    minimum, maximum = limits_by_command[command]
    if any(value < minimum or value > maximum for value in result):
        raise ValueError(f"{command} values must be in [{minimum}, {maximum}]")
    if spec.transport == "canfd" and command != "angles":
        if any(not value.is_integer() for value in result):
            raise ValueError(f"{command} values must be integers for {model}")
        return [int(value) for value in result]
    return result


class HandAdapter:
    """Uniform control and sensing API for all supported SDK hand models."""

    def __init__(
        self,
        model: str,
        *,
        side: str = "left",
        interface_name: str = "can0",
        interface_type: str = "socketcan",
        library_path: str | None = None,
        hand_factory: Callable[..., Any] = create_sdk_hand,
    ) -> None:
        self.model = normalize_model(model)
        self.spec = MODEL_SPECS[self.model]
        self._hand = hand_factory(
            self.model,
            side=side,
            interface_name=interface_name,
            interface_type=interface_type,
            library_path=library_path,
        )
        self._closed = False

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.spec.joint_names

    def _validate_command(
        self, values: Sequence[float], label: str
    ) -> list[float] | list[int]:
        return validate_command(self.model, values, label)

    def set_angles(self, values: Sequence[float]) -> list[float] | list[int]:
        normalized = self._validate_command(values, "angles")
        self._hand.angle.set_angles(normalized)
        return normalized

    def set_speeds(self, values: Sequence[float]) -> list[float] | list[int]:
        normalized = self._validate_command(values, "speeds")
        self._hand.speed.set_speeds(normalized)
        return normalized

    def set_torques(self, values: Sequence[float]) -> list[float] | list[int]:
        normalized = self._validate_command(values, "torques")
        self._hand.torque.set_torques(normalized)
        return normalized

    def read_angles(self, timeout_ms: float) -> list[float]:
        data = self._hand.angle.get_blocking(timeout_ms=timeout_ms)
        return _as_float_list(data.angles)

    def read_optional(self, manager_name: str, timeout_ms: float) -> list[float] | None:
        manager = getattr(self._hand, manager_name, None)
        if manager is None or not hasattr(manager, "get_blocking"):
            return None
        data = manager.get_blocking(timeout_ms=timeout_ms)
        for attribute in (
            manager_name + "s",
            manager_name,
            "values",
            "faults",
            "currents",
            "temperatures",
        ):
            if hasattr(data, attribute):
                return _as_float_list(getattr(data, attribute))
        return None

    def clear_faults(self) -> None:
        manager = getattr(self._hand, "fault", None)
        if manager is None:
            raise RuntimeError("fault manager is unavailable for this model")
        for method_name in ("clear_faults", "clear_all", "clear"):
            method = getattr(manager, method_name, None)
            if method is not None:
                method()
                return
        raise RuntimeError("fault clearing is not supported by this model")

    def close(self) -> None:
        if not self._closed:
            self._hand.close()
            self._closed = True
