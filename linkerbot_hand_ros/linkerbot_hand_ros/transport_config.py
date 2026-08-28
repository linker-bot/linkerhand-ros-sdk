"""Strict, backend-specific transport configuration parsed without I/O."""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any

from linkerbot_hand import ValidationError


class TransportBackend(str, Enum):
    """Transport implementations supported by the ROS configuration layer."""

    SOCKETCAN = "socketcan"
    PYTHON_CAN = "python_can"
    VENDOR_CANFD = "vendor_canfd"


@dataclass(frozen=True, slots=True)
class SocketCANConfiguration:
    """One Linux SocketCAN channel used for classic CAN or CAN FD."""

    channel: str
    nominal_bitrate: int | None = None
    data_bitrate: int | None = None
    auto_reconfigure: bool = False
    receive_own_messages: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "channel",
            _non_empty_string(self.channel, "channel"),
        )
        for name in ("nominal_bitrate", "data_bitrate"):
            _optional_positive_int(getattr(self, name), name)
        for name in ("auto_reconfigure", "receive_own_messages"):
            _boolean(getattr(self, name), name)

    @property
    def backend(self) -> TransportBackend:
        return TransportBackend.SOCKETCAN


@dataclass(frozen=True, slots=True)
class PythonCANConfiguration:
    """A python-can classic CAN backend other than the SocketCAN shorthand."""

    interface_name: str
    interface_type: str
    nominal_bitrate: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "interface_name",
            _non_empty_string(self.interface_name, "interface_name"),
        )
        object.__setattr__(
            self,
            "interface_type",
            _non_empty_string(self.interface_type, "interface_type"),
        )
        _optional_positive_int(self.nominal_bitrate, "nominal_bitrate")

    @property
    def backend(self) -> TransportBackend:
        return TransportBackend.PYTHON_CAN


@dataclass(frozen=True, slots=True)
class VendorCANFDConfiguration:
    """Vendor CAN FD adapter configuration using ``linkerbot-transport``."""

    device_index: int = 0
    channel_index: int = 0
    library_path: str | None = None
    nominal_bitrate: int | None = None
    data_bitrate: int | None = None
    config_flags: int = 0x07
    model: int = 0
    can_type: int = 1
    frame_type: int = 0x04

    def __post_init__(self) -> None:
        for name in ("device_index", "channel_index"):
            _non_negative_int(getattr(self, name), name)
        if self.library_path is not None:
            object.__setattr__(
                self,
                "library_path",
                _non_empty_string(self.library_path, "library_path"),
            )
        for name in ("nominal_bitrate", "data_bitrate"):
            _optional_positive_int(getattr(self, name), name)
        for name in ("config_flags", "model", "can_type", "frame_type"):
            _byte(getattr(self, name), name)

    @property
    def backend(self) -> TransportBackend:
        return TransportBackend.VENDOR_CANFD


TransportConfiguration = (
    SocketCANConfiguration | PythonCANConfiguration | VendorCANFDConfiguration
)


def parse_transport_configuration(raw: object) -> TransportConfiguration:
    """Parse one strict transport JSON object without constructing a transport."""

    if not isinstance(raw, str):
        raise ValidationError("transport_json must be a string")
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as error:
        raise ValidationError(f"transport_json is invalid JSON: {error.msg}") from error
    if not isinstance(parsed, dict):
        raise ValidationError("transport_json must contain a JSON object")
    options = dict(parsed)
    backend_value = options.pop("backend", None)
    try:
        backend = TransportBackend(backend_value)
    except (TypeError, ValueError) as error:
        choices = ", ".join(value.value for value in TransportBackend)
        raise ValidationError(f"transport backend must be one of: {choices}") from error

    if backend is TransportBackend.SOCKETCAN:
        return _parse_socketcan(options)
    if backend is TransportBackend.PYTHON_CAN:
        return _parse_python_can(options)
    return _parse_vendor_canfd(options)


def _parse_socketcan(options: dict[str, Any]) -> SocketCANConfiguration:
    _reject_unknown(
        options,
        {
            "channel",
            "nominal_bitrate",
            "data_bitrate",
            "auto_reconfigure",
            "receive_own_messages",
        },
    )
    if "channel" not in options:
        raise ValidationError("socketcan transport requires channel")
    return SocketCANConfiguration(**options)


def _parse_python_can(options: dict[str, Any]) -> PythonCANConfiguration:
    _reject_unknown(
        options,
        {"interface_name", "interface_type", "nominal_bitrate"},
    )
    missing = {"interface_name", "interface_type"}.difference(options)
    if missing:
        names = ", ".join(sorted(missing))
        raise ValidationError(f"python_can transport requires: {names}")
    return PythonCANConfiguration(**options)


def _parse_vendor_canfd(options: dict[str, Any]) -> VendorCANFDConfiguration:
    _reject_unknown(
        options,
        {
            "device_index",
            "channel_index",
            "library_path",
            "nominal_bitrate",
            "data_bitrate",
            "config_flags",
            "model",
            "can_type",
            "frame_type",
        },
    )
    return VendorCANFDConfiguration(**options)


def _reject_unknown(options: Mapping[str, object], allowed: set[str]) -> None:
    unknown = set(options).difference(allowed)
    if unknown:
        names = ", ".join(sorted(unknown))
        raise ValidationError(f"transport_json contains unknown keys: {names}")


def _non_empty_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValidationError(f"{name} must be a non-empty string")
    return value.strip()


def _optional_positive_int(value: object, name: str) -> int | None:
    if value is None:
        return None
    if type(value) is not int or value <= 0:
        raise ValidationError(f"{name} must be a positive int or null")
    return value


def _non_negative_int(value: object, name: str) -> int:
    if type(value) is not int or value < 0:
        raise ValidationError(f"{name} must be a non-negative int")
    return value


def _byte(value: object, name: str) -> int:
    if type(value) is not int or not 0 <= value <= 0xFF:
        raise ValidationError(f"{name} must be an integer between 0 and 255")
    return value


def _boolean(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValidationError(f"{name} must be a boolean")
    return value


__all__ = [
    "PythonCANConfiguration",
    "SocketCANConfiguration",
    "TransportBackend",
    "TransportConfiguration",
    "VendorCANFDConfiguration",
    "parse_transport_configuration",
]
