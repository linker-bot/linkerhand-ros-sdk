"""Management worker and service contracts for HandDevice 1.0."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from fakes_v1 import DeviceBuilder
from linkerbot_hand import CommandSemantic, ErrorCode, LifecycleError, TimeoutError
from linkerbot_hand_ros.admin_worker import AdminWorker
from linkerbot_hand_ros.command_router import DiscreteOperation
from linkerbot_hand_ros.node import LinkerbotHandNode
from rclpy.parameter import Parameter

from linkerbot_hand_msgs.srv import (
    GetDeviceInfo,
    Recover,
    SetRawValues,
    SubmitCommand,
)


@pytest.fixture
def active_node():
    rclpy.init()
    builder = DeviceBuilder()
    node = LinkerbotHandNode(device_factory=builder)
    node.trigger_configure()
    node.trigger_activate()
    try:
        yield node, builder
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_device_info_read_runs_on_admin_worker(active_node):
    node, builder = active_node
    response = node._interfaces.management._get_device_info(
        GetDeviceInfo.Request(timeout_s=0.2),
        GetDeviceInfo.Response(),
    )

    assert response.success
    assert response.info.model == "L6"
    assert builder.latest_runtime.device_info_threads == ["linkerbot_hand-admin"]
    assert threading.current_thread().name != "linkerbot_hand-admin"


def test_recover_service_preserves_stable_error_fields(active_node):
    node, builder = active_node
    success = node._interfaces.management._recover(
        Recover.Request(timeout_s=0.2),
        Recover.Response(),
    )
    too_long = node._interfaces.management._recover(
        Recover.Request(timeout_s=1.9),
        Recover.Response(),
    )

    assert success.success
    assert builder.latest_runtime.recover_count == 1
    assert not too_long.success
    assert too_long.error_code == ErrorCode.VALIDATION.value


def test_discrete_service_returns_at_sdk_acceptance(active_node):
    node, builder = active_node
    response = node._interfaces.management._submit_discrete(
        DiscreteOperation.CLEAR_FAULTS,
        SubmitCommand.Request(),
        SubmitCommand.Response(),
    )

    assert response.accepted
    assert response.command_id > 0
    assert builder.latest_runtime.operations == ["clear_faults"]


def test_o6_save_parameters_service_tracks_acknowledged_operation(monkeypatch):
    rclpy.init()
    builder = DeviceBuilder()
    node = LinkerbotHandNode(device_factory=builder)
    try:
        result = node.set_parameters_atomically(
            [Parameter("model", value="o6"), Parameter("side", value="right")]
        )
        assert result.successful
        node.trigger_configure()
        node.trigger_activate()

        response = node._interfaces.management._submit_discrete(
            DiscreteOperation.SAVE_PARAMETERS,
            SubmitCommand.Request(),
            SubmitCommand.Response(),
        )
        messages = []
        with monkeypatch.context() as patch:
            patch.setattr(
                node._interfaces.commands,
                "_status_publisher",
                SimpleNamespace(publish=messages.append),
            )
            node._publish_command_status()

        assert response.accepted
        assert builder.latest_runtime.operations == ["save_parameters"]
        assert len(messages) == 1
        assert messages[0].stage == messages[0].STAGE_ACKNOWLEDGED
        assert messages[0].ack_support == messages[0].ACK_SUPPORTED
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize(
    "name, semantic",
    [
        ("stall_thresholds", CommandSemantic.STALL_THRESHOLDS),
        ("stall_detection_times", CommandSemantic.STALL_DETECTION_TIMES),
        ("stall_hold_currents", CommandSemantic.STALL_HOLD_CURRENTS),
    ],
)
def test_o6_stall_write_uses_latest_protocol_semantics_without_waiting(
    name,
    semantic,
):
    rclpy.init()
    builder = DeviceBuilder()
    node = LinkerbotHandNode(device_factory=builder)
    try:
        result = node.set_parameters_atomically(
            [Parameter("model", value="o6"), Parameter("side", value="right")]
        )
        assert result.successful
        node.trigger_configure()
        node.trigger_activate()
        runtime = builder.latest_runtime
        command = runtime.capabilities.command(semantic)
        assert command is not None
        values = [
            float(command.range_for(index).minimum)
            for index in range(len(command.channels))
        ]

        started = time.monotonic()
        response = node._interfaces.management._set_raw_values(
            name,
            SetRawValues.Request(user_sequence=8, raw=values),
            SetRawValues.Response(),
        )

        assert time.monotonic() - started < 0.05
        assert response.accepted
        assert runtime.submissions[-1] == (
            semantic,
            tuple(int(value) for value in values),
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_admin_worker_timeout_and_bounded_stop():
    worker = AdminWorker(capacity=1, name="admin-test")
    release = threading.Event()
    worker.start()
    try:
        with pytest.raises(TimeoutError):
            worker.call(
                "blocked",
                lambda: release.wait(0.2),
                timeout_s=0.01,
            )
        with pytest.raises(LifecycleError):
            worker.stop(timeout_s=0.01)
    finally:
        release.set()
        worker.stop(timeout_s=0.5)


def test_inactive_management_service_does_not_touch_device():
    rclpy.init()
    builder = DeviceBuilder()
    node = LinkerbotHandNode(device_factory=builder)
    try:
        node.trigger_configure()
        response = node._interfaces.management._get_device_info(
            GetDeviceInfo.Request(timeout_s=0.2),
            GetDeviceInfo.Response(),
        )

        assert not response.success
        assert response.error_code == ErrorCode.DEVICE_NOT_READY.value
        assert builder.runtime_factories[-1].calls == 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
