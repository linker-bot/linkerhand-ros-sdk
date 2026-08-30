from dataclasses import dataclass, field

import pytest
from linkerbot_hand import HandDevice, ValidationError, get_declared_capabilities
from linkerbot_hand_ros.driver_config import read_driver_configuration
from linkerbot_hand_ros.transport_config import (
    PythonCANConfiguration,
    SocketCANConfiguration,
    VendorCANFDConfiguration,
)
from linkerbot_hand_ros.transport_factory import (
    TransportFactories,
    create_hand_device,
    create_transport,
)


@dataclass
class ConstructorSpy:
    calls: list[tuple[tuple[object, ...], dict[str, object]]] = field(
        default_factory=list
    )
    result: object = field(default_factory=object)

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return self.result


class NoIOTransport:
    def __init__(self):
        self.io_calls = 0

    def open(self):
        self.io_calls += 1
        raise AssertionError("configure must not open a transport")

    def send(self, *_args, **_kwargs):
        self.io_calls += 1
        raise AssertionError("configure must not send")

    def receive(self, *_args, **_kwargs):
        self.io_calls += 1
        raise AssertionError("configure must not receive")


def _capabilities(model="L6"):
    return get_declared_capabilities(
        model,
        **({} if model == "L30" else {"side": "left"}),
    )


def _driver_parameters(**overrides):
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
        "angle_feedback": "adaptive",
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


def test_socketcan_selects_classic_can_without_opening_it():
    classic = ConstructorSpy(result=NoIOTransport())
    socketcan_fd = ConstructorSpy()
    vendor = ConstructorSpy()

    transport = create_transport(
        SocketCANConfiguration("can1"),
        _capabilities("L6"),
        factories=TransportFactories(classic, socketcan_fd, vendor),
    )

    assert transport is classic.result
    assert classic.calls == [(("can1", "socketcan"), {})]
    assert not socketcan_fd.calls
    assert not vendor.calls
    assert transport.io_calls == 0


def test_socketcan_fd_uses_model_declared_bitrates():
    classic = ConstructorSpy()
    socketcan_fd = ConstructorSpy(result=NoIOTransport())

    create_transport(
        SocketCANConfiguration("can0"),
        _capabilities("O20"),
        factories=TransportFactories(classic, socketcan_fd, ConstructorSpy()),
    )

    assert socketcan_fd.calls == [
        (
            ("can0",),
            {
                "bitrate": 1_000_000,
                "data_bitrate": 5_000_000,
                "auto_reconfigure": False,
                "receive_own_messages": False,
            },
        )
    ]
    assert not classic.calls


def test_transport_factory_rejects_bitrate_and_backend_conflicts():
    with pytest.raises(ValidationError, match="conflicts with O30i capability"):
        create_transport(
            SocketCANConfiguration("can0", data_bitrate=5_000_000),
            _capabilities("O30i"),
            factories=TransportFactories(
                ConstructorSpy(), ConstructorSpy(), ConstructorSpy()
            ),
        )

    with pytest.raises(ValidationError, match="only valid for classic CAN"):
        create_transport(
            PythonCANConfiguration("can0", "socketcan"),
            _capabilities("L30"),
            factories=TransportFactories(
                ConstructorSpy(), ConstructorSpy(), ConstructorSpy()
            ),
        )

    with pytest.raises(ValidationError, match="only valid for CAN FD"):
        create_transport(
            VendorCANFDConfiguration(),
            _capabilities("L6"),
            factories=TransportFactories(
                ConstructorSpy(), ConstructorSpy(), ConstructorSpy()
            ),
        )


def test_vendor_canfd_preserves_adapter_configuration():
    vendor = ConstructorSpy(result=NoIOTransport())
    transport = create_transport(
        VendorCANFDConfiguration(
            device_index=2,
            channel_index=1,
            library_path="/opt/linker/libcanbus.so",
            frame_type=0x0C,
        ),
        _capabilities("L30"),
        factories=TransportFactories(ConstructorSpy(), ConstructorSpy(), vendor),
    )

    assert transport is vendor.result
    args, kwargs = vendor.calls[0]
    assert args == (2, 1, "/opt/linker/libcanbus.so")
    assert kwargs["config"].nom_baud == 1_000_000
    assert kwargs["config"].dat_baud == 5_000_000
    assert kwargs["config"].frame_type == 0x0C


def test_hand_device_construction_performs_zero_transport_io():
    no_io_transport = NoIOTransport()
    classic = ConstructorSpy(result=no_io_transport)
    configuration = read_driver_configuration(_driver_parameters().__getitem__)

    hand = create_hand_device(
        configuration,
        transport_factories=TransportFactories(
            classic, ConstructorSpy(), ConstructorSpy()
        ),
    )

    assert isinstance(hand, HandDevice)
    assert hand.get_capabilities().model == "L6"
    assert hand.get_health().connected is False
    assert hand.get_snapshot().connection_generation == 0
    assert no_io_transport.io_calls == 0


def test_l30_device_factory_does_not_receive_a_fake_side():
    configuration = read_driver_configuration(
        _driver_parameters(
            model="l30",
            side="",
            transport_json=(
                '{"backend":"vendor_canfd","device_index":0,"channel_index":0}'
            ),
        ).__getitem__
    )
    device = ConstructorSpy(result=object())

    result = create_hand_device(
        configuration,
        transport_factories=TransportFactories(
            ConstructorSpy(), ConstructorSpy(), ConstructorSpy()
        ),
        device_factory=device,
    )

    assert result is device.result
    _args, kwargs = device.calls[0]
    assert kwargs["model"] == "L30"
    assert "side" not in kwargs


def test_equivalent_real_transport_descriptors_share_the_same_pool_key():
    first = create_transport(SocketCANConfiguration("can9"), _capabilities("L6"))
    second = create_transport(SocketCANConfiguration("can9"), _capabilities("L6"))

    assert first is not second
    assert first.pool_key == second.pool_key
