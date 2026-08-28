"""Hand SDK 1.0 fakes used by ROS lifecycle contract tests."""

from __future__ import annotations

import threading
import time
from dataclasses import replace

from linkerbot_hand import (
    AckSupport,
    CommandFamily,
    CommandHandle,
    CommandSemantic,
    ConnectionState,
    DataOrigin,
    DeviceInfo,
    HandDevice,
    HandSnapshot,
    HealthStatus,
    LifecycleState,
    PollingPlan,
    SampleMeta,
    TelemetrySemantic,
    VersionField,
)
from linkerbot_hand_ros.driver_config import DriverConfiguration

_FAMILIES = {
    CommandSemantic.ANGLES: CommandFamily.ANGLE_TARGET,
    CommandSemantic.SPEEDS: CommandFamily.SPEED_SETTING,
    CommandSemantic.ACCELERATIONS: CommandFamily.ACCELERATION_SETTING,
    CommandSemantic.CURRENT_LIMITS: CommandFamily.CURRENT_LIMIT,
    CommandSemantic.EFFORT_LIMITS: CommandFamily.EFFORT_LIMIT,
    CommandSemantic.TORQUES: CommandFamily.TORQUE_TARGET,
    CommandSemantic.MOTION_TIMES: CommandFamily.MOTION_TIME_SETTING,
    CommandSemantic.STALL_DETECTION_TIMES: CommandFamily.STALL_CONFIGURATION,
    CommandSemantic.STALL_THRESHOLDS: CommandFamily.STALL_CONFIGURATION,
    CommandSemantic.STALL_HOLD_CURRENTS: CommandFamily.STALL_CONFIGURATION,
}
_SNAPSHOT_ATTRIBUTES = {
    TelemetrySemantic.ANGLES: "angles",
    TelemetrySemantic.MEASURED_SPEEDS: "measured_speeds",
    TelemetrySemantic.CONFIGURED_SPEEDS: "configured_speeds",
    TelemetrySemantic.CONFIGURED_ACCELERATIONS: "configured_accelerations",
    TelemetrySemantic.CURRENTS: "currents",
    TelemetrySemantic.CURRENT_LIMITS: "current_limits",
    TelemetrySemantic.EFFORT_LIMITS: "effort_limits",
    TelemetrySemantic.TORQUES: "torques",
    TelemetrySemantic.TEMPERATURES: "temperatures",
    TelemetrySemantic.FORCE: "force",
    TelemetrySemantic.FAULTS: "faults",
    TelemetrySemantic.VOLTAGES: "voltages",
    TelemetrySemantic.MOTION_TIMES: "motion_times",
    TelemetrySemantic.STALL_DETECTION_TIMES: "stall_detection_times",
    TelemetrySemantic.STALL_THRESHOLDS: "stall_thresholds",
    TelemetrySemantic.STALL_HOLD_CURRENTS: "stall_hold_currents",
}


class FakeRuntime:
    """Synchronous in-memory runtime implementing the HandDevice boundary."""

    def __init__(self, **options: object) -> None:
        declared = options["capabilities"]
        self.capabilities = replace(
            declared,
            identity=replace(
                declared.identity,
                verified=True,
                firmware="test-1.0",
            ),
        )
        self.snapshot: HandSnapshot = options["initial_snapshot"]
        self.health: HealthStatus = options["initial_health"]
        self.open_count = 0
        self.close_count = 0
        self.stop_polling_count = 0
        self.polling_plans: list[PollingPlan] = []
        self.submissions: list[tuple[CommandSemantic, tuple[int | float, ...]]] = []
        self.operations: list[str] = []
        self.recover_count = 0
        self.read_count = 0
        self.snapshot_get_count = 0
        self.health_get_count = 0
        self.capabilities_get_count = 0
        self.device_info_threads: list[str] = []
        self.open_error: Exception | None = None
        self._opened = False
        self._polling = False
        self._next_command_id = 1
        self.device_info = DeviceInfo(
            model=self.capabilities.model,
            serial_number="TEST-001",
            pcb_version=None,
            firmware_version=VersionField(raw="test-1.0", text="test-1.0"),
            mechanical_version=None,
            product_code=None,
            sensor_variant=None,
            side=self.capabilities.side,
            raw_identity=b"TEST",
            meta=SampleMeta(
                sequence=1,
                collected_at=time.monotonic(),
                age_s=0.0,
                valid=True,
                stale=False,
                connection_generation=1,
                origin=DataOrigin.DEVICE_RESPONSE,
                within_declared_range=True,
            ),
        )

    def open(self) -> None:
        if self._opened:
            return
        if self.open_error is not None:
            raise self.open_error
        self.open_count += 1
        self._opened = True
        self.snapshot = replace(
            self.snapshot,
            connection_generation=1,
            device_info=self.device_info,
        )
        self.health = replace(
            self.health,
            lifecycle_state=LifecycleState.OPEN,
            connection_state=ConnectionState.CONNECTED,
            connected=True,
            connection_generation=1,
            identity_verified=True,
        )

    def close(self) -> None:
        if not self._opened:
            return
        self.close_count += 1
        self._opened = False
        self._polling = False
        self.health = replace(
            self.health,
            lifecycle_state=LifecycleState.CLOSED,
            connection_state=ConnectionState.CLOSED,
            connected=False,
            polling_active=False,
        )

    def start_polling(self, plan: PollingPlan) -> None:
        self._require_open()
        if self.polling_plans and self.polling_plans[-1] == plan and self._polling:
            return
        self.polling_plans.append(plan)
        self._polling = True
        self.health = replace(self.health, polling_active=True)

    def stop_polling(self) -> None:
        if not self._opened or not self._polling:
            return
        self.stop_polling_count += 1
        self._polling = False
        self.health = replace(self.health, polling_active=False)

    def submit(
        self,
        semantic: CommandSemantic,
        values: tuple[int | float, ...],
    ) -> CommandHandle:
        self._require_open()
        self.submissions.append((semantic, values))
        handle = CommandHandle(
            self._next_command_id,
            _FAMILIES[semantic],
            ack_support=AckSupport.UNSUPPORTED,
            accepted_at=time.monotonic(),
        )
        self._next_command_id += 1
        handle._mark_dispatched(timestamp=time.monotonic(), sent_frames=1)
        return handle

    def read(self, semantic: TelemetrySemantic, _timeout: float) -> object:
        self._require_open()
        self.read_count += 1
        return getattr(self.snapshot, _SNAPSHOT_ATTRIBUTES[semantic])

    def read_device_info(self, _timeout: float) -> DeviceInfo:
        self._require_open()
        self.device_info_threads.append(threading.current_thread().name)
        self.snapshot = replace(self.snapshot, device_info=self.device_info)
        return self.device_info

    def operate(self, operation: str) -> CommandHandle:
        self._require_open()
        self.operations.append(operation)
        handle = CommandHandle(
            self._next_command_id,
            CommandFamily.DISCRETE_OPERATION,
            ack_support=AckSupport.UNSUPPORTED,
            accepted_at=time.monotonic(),
        )
        self._next_command_id += 1
        handle._mark_dispatched(timestamp=time.monotonic(), sent_frames=1)
        return handle

    def recover(self, _timeout: float) -> None:
        self._require_open()
        self.recover_count += 1

    def get_capabilities(self):
        self.capabilities_get_count += 1
        return self.capabilities

    def get_snapshot(self):
        self.snapshot_get_count += 1
        return self.snapshot

    def get_health(self):
        self.health_get_count += 1
        return self.health

    def _require_open(self) -> None:
        if not self._opened:
            raise RuntimeError("fake runtime is closed")


class RuntimeFactory:
    def __init__(self) -> None:
        self.calls = 0
        self.runtime: FakeRuntime | None = None
        self.open_error: Exception | None = None

    def __call__(self, **options: object) -> FakeRuntime:
        self.calls += 1
        self.runtime = FakeRuntime(**options)
        self.runtime.open_error = self.open_error
        self.open_error = None
        return self.runtime


class DeviceBuilder:
    """Construct real HandDevice facades while recording injected runtimes."""

    def __init__(self) -> None:
        self.calls: list[DriverConfiguration] = []
        self.runtime_factories: list[RuntimeFactory] = []
        self.next_open_error: Exception | None = None

    def __call__(self, configuration: DriverConfiguration) -> HandDevice:
        factory = RuntimeFactory()
        factory.open_error = self.next_open_error
        self.next_open_error = None
        self.calls.append(configuration)
        self.runtime_factories.append(factory)
        options = {
            "model": configuration.model,
            "transport": object(),
            "config": configuration.device_config,
            "runtime_factory": factory,
        }
        if configuration.side is not None:
            options["side"] = configuration.side
        return HandDevice(**options)

    @property
    def latest_runtime(self) -> FakeRuntime:
        runtime = self.runtime_factories[-1].runtime
        assert runtime is not None
        return runtime


__all__ = ["DeviceBuilder", "FakeRuntime", "RuntimeFactory"]
