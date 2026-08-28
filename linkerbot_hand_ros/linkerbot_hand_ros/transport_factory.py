"""Construct transport descriptors and ``HandDevice`` without device I/O."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from linkerbot_hand import (
    DeviceCapabilities,
    HandDevice,
    HandSide,
    TransportKind,
    ValidationError,
)
from linkerbot_transport.can import CANTransport
from linkerbot_transport.canfd import (
    CANFDConfigOptions,
    CANFDTransport,
    SocketCANFDTransport,
)

from .driver_config import DriverConfiguration
from .transport_config import (
    PythonCANConfiguration,
    SocketCANConfiguration,
    TransportConfiguration,
    VendorCANFDConfiguration,
)

TransportConstructor = Callable[..., object]
DeviceConstructor = Callable[..., HandDevice]


@dataclass(frozen=True, slots=True)
class TransportFactories:
    """Injectable constructors used to prove configure performs no I/O."""

    classic_can: TransportConstructor = CANTransport
    socketcan_fd: TransportConstructor = SocketCANFDTransport
    vendor_canfd: TransportConstructor = CANFDTransport


def create_transport(
    configuration: TransportConfiguration,
    capabilities: DeviceCapabilities,
    *,
    factories: TransportFactories | None = None,
) -> object:
    """Create a transport descriptor compatible with SDK capabilities."""

    if not isinstance(capabilities, DeviceCapabilities):
        raise ValidationError("capabilities must be DeviceCapabilities")
    constructors = TransportFactories() if factories is None else factories
    if not isinstance(constructors, TransportFactories):
        raise ValidationError("factories must be TransportFactories or None")

    if isinstance(configuration, SocketCANConfiguration):
        return _create_socketcan(configuration, capabilities, constructors)
    if isinstance(configuration, PythonCANConfiguration):
        return _create_python_can(configuration, capabilities, constructors)
    if isinstance(configuration, VendorCANFDConfiguration):
        return _create_vendor_canfd(configuration, capabilities, constructors)
    raise ValidationError("unsupported transport configuration")


def create_hand_device(
    configuration: DriverConfiguration,
    *,
    transport_factories: TransportFactories | None = None,
    device_factory: DeviceConstructor = HandDevice,
) -> HandDevice:
    """Construct the single public SDK facade without opening the transport."""

    if not isinstance(configuration, DriverConfiguration):
        raise ValidationError("configuration must be DriverConfiguration")
    if not callable(device_factory):
        raise ValidationError("device_factory must be callable")
    capabilities = configuration.declared_capabilities
    transport = create_transport(
        configuration.transport,
        capabilities,
        factories=transport_factories,
    )
    options: dict[str, Any] = {
        "model": capabilities.model,
        "transport": transport,
        "config": configuration.device_config,
    }
    if capabilities.side is not HandSide.NOT_APPLICABLE:
        options["side"] = capabilities.side.value
    return device_factory(**options)


def _create_socketcan(
    configuration: SocketCANConfiguration,
    capabilities: DeviceCapabilities,
    factories: TransportFactories,
) -> object:
    nominal_bitrate = _declared_bitrate(
        "nominal_bitrate",
        configuration.nominal_bitrate,
        capabilities.transport.nominal_bitrate,
        capabilities.model,
    )
    if capabilities.transport.kind is TransportKind.CAN:
        if configuration.data_bitrate is not None:
            raise ValidationError(
                "data_bitrate is invalid for a classic CAN hand",
                model=capabilities.model,
            )
        if configuration.auto_reconfigure or configuration.receive_own_messages:
            raise ValidationError(
                "CAN FD SocketCAN options are invalid for a classic CAN hand",
                model=capabilities.model,
            )
        # Classic python-can uses the bitrate configured on the Linux interface.
        # The value above is still checked against the SDK source of truth.
        _ = nominal_bitrate
        return factories.classic_can(configuration.channel, "socketcan")
    if capabilities.transport.kind is not TransportKind.CAN_FD:
        raise ValidationError(
            f"SocketCAN does not support transport kind "
            f"{capabilities.transport.kind.value}",
            model=capabilities.model,
        )
    data_bitrate = _declared_bitrate(
        "data_bitrate",
        configuration.data_bitrate,
        capabilities.transport.data_bitrate,
        capabilities.model,
    )
    return factories.socketcan_fd(
        configuration.channel,
        bitrate=nominal_bitrate,
        data_bitrate=data_bitrate,
        auto_reconfigure=configuration.auto_reconfigure,
        receive_own_messages=configuration.receive_own_messages,
    )


def _create_python_can(
    configuration: PythonCANConfiguration,
    capabilities: DeviceCapabilities,
    factories: TransportFactories,
) -> object:
    if capabilities.transport.kind is not TransportKind.CAN:
        raise ValidationError(
            "python_can is only valid for classic CAN hand profiles",
            model=capabilities.model,
        )
    _declared_bitrate(
        "nominal_bitrate",
        configuration.nominal_bitrate,
        capabilities.transport.nominal_bitrate,
        capabilities.model,
    )
    return factories.classic_can(
        configuration.interface_name,
        configuration.interface_type,
    )


def _create_vendor_canfd(
    configuration: VendorCANFDConfiguration,
    capabilities: DeviceCapabilities,
    factories: TransportFactories,
) -> object:
    if capabilities.transport.kind is not TransportKind.CAN_FD:
        raise ValidationError(
            "vendor_canfd is only valid for CAN FD hand profiles",
            model=capabilities.model,
        )
    nominal_bitrate = _declared_bitrate(
        "nominal_bitrate",
        configuration.nominal_bitrate,
        capabilities.transport.nominal_bitrate,
        capabilities.model,
    )
    data_bitrate = _declared_bitrate(
        "data_bitrate",
        configuration.data_bitrate,
        capabilities.transport.data_bitrate,
        capabilities.model,
    )
    vendor_options = CANFDConfigOptions(
        nom_baud=nominal_bitrate,
        dat_baud=data_bitrate,
        config=configuration.config_flags,
        model=configuration.model,
        cantype=configuration.can_type,
        frame_type=configuration.frame_type,
    )
    return factories.vendor_canfd(
        configuration.device_index,
        configuration.channel_index,
        configuration.library_path,
        config=vendor_options,
    )


def _declared_bitrate(
    name: str,
    configured: int | None,
    declared: int | None,
    model: str,
) -> int:
    if declared is None:
        raise ValidationError(
            f"{model} does not declare {name}",
            model=model,
        )
    if configured is not None and configured != declared:
        raise ValidationError(
            f"{name} {configured} conflicts with {model} capability {declared}",
            model=model,
        )
    return declared


__all__ = [
    "TransportFactories",
    "create_hand_device",
    "create_transport",
]
