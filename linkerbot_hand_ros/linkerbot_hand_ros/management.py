"""Reliable low-frequency ROS management services for one hand device."""

# Service boundaries must translate every SDK/backend exception to stable fields.
# ruff: noqa: BLE001

from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import Any

from linkerbot_hand import (
    CommandSemantic,
    DeviceCapabilities,
    ErrorCode,
    HandDevice,
    HandError,
    HandSnapshot,
    TimeoutError,
    ValidationError,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode

from linkerbot_hand_msgs.msg import HandRawCommand
from linkerbot_hand_msgs.srv import (
    GetDeviceInfo,
    GetStallProtection,
    Recover,
    SetRawValues,
    SubmitCommand,
)

from .admin_worker import AdminWorker
from .command_router import (
    DiscreteOperation,
    RoutedCommand,
    route_discrete_operation,
    route_raw_command,
)
from .conversion import device_info_to_message, stall_protection_to_message

_STALL_SETTERS = {
    "stall_detection_times": CommandSemantic.STALL_DETECTION_TIMES,
    "stall_thresholds": CommandSemantic.STALL_THRESHOLDS,
    "stall_hold_currents": CommandSemantic.STALL_HOLD_CURRENTS,
}


class ManagementInterface:
    """Own bounded admin execution and reliable service endpoints."""

    def __init__(
        self,
        node: LifecycleNode,
        *,
        capabilities: DeviceCapabilities,
        device: Callable[[], HandDevice],
        is_active: Callable[[], bool],
        track: Callable[[RoutedCommand, float | None], None],
        shutdown_timeout_s: float,
        worker_capacity: int,
    ) -> None:
        if not isinstance(capabilities, DeviceCapabilities):
            raise TypeError("capabilities must be DeviceCapabilities")
        self._node = node
        self._capabilities = capabilities
        self._device = device
        self._is_active = is_active
        self._track = track
        self._shutdown_timeout_s = shutdown_timeout_s
        self._worker = AdminWorker(
            capacity=worker_capacity,
            name=f"{node.get_name()}-admin",
        )
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self._services: list[Any] = []
        try:
            self._create_services()
        except Exception:
            for service in self._services:
                self._node.destroy_service(service)
            self._services.clear()
            raise

    def activate(self) -> None:
        """Enable bounded active-read work for the lifecycle active state."""

        self._worker.start()

    def deactivate(self) -> None:
        """Stop the worker before the SDK connection can be closed."""

        self._worker.stop(timeout_s=self._shutdown_timeout_s)

    def destroy(self) -> None:
        """Stop work and destroy every service entity."""

        stop_error: Exception | None = None
        try:
            self.deactivate()
        except Exception as error:
            stop_error = error
        for service in self._services:
            self._node.destroy_service(service)
        self._services.clear()
        if stop_error is not None:
            raise stop_error

    @property
    def worker_active(self) -> bool:
        return self._worker.active

    def _create_services(self) -> None:
        operations = self._capabilities.operations
        if operations.read_device_info:
            self._add_service(
                GetDeviceInfo,
                "~/get_device_info",
                self._get_device_info,
            )
        if operations.read_stall_protection:
            self._add_service(
                GetStallProtection,
                "~/get_stall_protection",
                self._get_stall_protection,
            )
        self._add_service(
            Recover,
            "~/recover",
            self._recover,
        )
        if operations.write_stall_protection:
            for name in _STALL_SETTERS:
                self._add_service(
                    SetRawValues,
                    f"~/set_{name}",
                    lambda request, response, value=name: self._set_raw_values(
                        value,
                        request,
                        response,
                    ),
                )
        for operation in DiscreteOperation:
            if not getattr(operations, operation.value):
                continue
            self._add_service(
                SubmitCommand,
                f"~/{operation.value}",
                lambda request, response, value=operation: self._submit_discrete(
                    value,
                    request,
                    response,
                ),
            )

    def _add_service(self, service_type: Any, name: str, callback: Any) -> None:
        service = self._node.create_service(
            service_type,
            name,
            callback,
            callback_group=self._callback_group,
        )
        self._services.append(service)

    def _get_device_info(
        self,
        request: GetDeviceInfo.Request,
        response: GetDeviceInfo.Response,
    ) -> GetDeviceInfo.Response:
        if not self._require_active(response):
            return response
        try:
            timeout = self._request_timeout(request.timeout_s)
            info = self._worker.call(
                "get_device_info",
                lambda: self._device().get_device_info(timeout),
                timeout_s=_worker_timeout(timeout),
            )
            response.success = True
            response.info = device_info_to_message(
                info,
                ros_now_ns=self._node.get_clock().now().nanoseconds,
            )
            response.message = "device information refreshed"
        except Exception as error:
            _set_failure(response, error)
        return response

    def _get_stall_protection(
        self,
        request: GetStallProtection.Request,
        response: GetStallProtection.Response,
    ) -> GetStallProtection.Response:
        if not self._require_active(response):
            return response
        try:
            timeout = self._request_timeout(request.timeout_s)
            snapshot = self._worker.call(
                "get_stall_protection",
                lambda: self._read_stall_protection(timeout),
                timeout_s=_worker_timeout(timeout),
            )
            message = stall_protection_to_message(
                snapshot,
                self._device().get_capabilities(),
                ros_now_ns=self._node.get_clock().now().nanoseconds,
            )
            if message is None:
                raise RuntimeError("stall protection cache is unavailable")
            response.success = True
            response.state = message
            response.message = "stall protection refreshed"
        except Exception as error:
            _set_failure(response, error)
        return response

    def _recover(
        self,
        request: Recover.Request,
        response: Recover.Response,
    ) -> Recover.Response:
        if not self._require_active(response):
            return response
        try:
            timeout = self._request_timeout(request.timeout_s)
            self._worker.call(
                "recover",
                lambda: self._device().recover(timeout),
                timeout_s=_worker_timeout(timeout),
            )
            response.success = True
            response.message = "identity-verified recovery completed"
        except Exception as error:
            _set_failure(response, error)
        return response

    def _set_raw_values(
        self,
        name: str,
        request: SetRawValues.Request,
        response: SetRawValues.Response,
    ) -> SetRawValues.Response:
        driver_received_at = time.monotonic()
        if not self._require_active(response, accepted_field=True):
            return response
        try:
            routed = route_raw_command(
                self._device(),
                _STALL_SETTERS[name],
                HandRawCommand(
                    user_sequence=request.user_sequence,
                    raw=request.raw,
                ),
                require_client_id=False,
            )
            self._track(routed, driver_received_at)
            response.accepted = True
            response.command_id = routed.handle.command_id
            response.message = "command accepted; delivery is asynchronous"
        except Exception as error:
            _set_failure(response, error, accepted_field=True)
        return response

    def _submit_discrete(
        self,
        operation: DiscreteOperation,
        _request: SubmitCommand.Request,
        response: SubmitCommand.Response,
    ) -> SubmitCommand.Response:
        driver_received_at = time.monotonic()
        if not self._require_active(response, accepted_field=True):
            return response
        try:
            routed = route_discrete_operation(self._device(), operation)
            self._track(routed, driver_received_at)
            response.accepted = True
            response.command_id = routed.handle.command_id
            response.message = "command accepted; delivery is asynchronous"
        except Exception as error:
            _set_failure(response, error, accepted_field=True)
        return response

    def _read_stall_protection(self, timeout_s: float) -> HandSnapshot:
        deadline = time.monotonic() + timeout_s
        device = self._device()
        for getter in (
            device.get_stall_detection_times,
            device.get_stall_thresholds,
            device.get_stall_hold_currents,
        ):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    "stall protection read exceeded its overall deadline",
                    operation="get_stall_protection",
                )
            getter(remaining)
        return device.get_snapshot()

    def _request_timeout(self, value: object) -> float:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValidationError("timeout_s must be finite and positive")
        timeout = float(value)
        if _worker_timeout(timeout) >= self._shutdown_timeout_s:
            raise ValidationError(
                "service timeout must leave time for bounded lifecycle shutdown"
            )
        return timeout

    def _require_active(
        self,
        response: Any,
        *,
        accepted_field: bool = False,
    ) -> bool:
        if self._is_active():
            return True
        if accepted_field:
            response.accepted = False
        else:
            response.success = False
        response.error_code = ErrorCode.DEVICE_NOT_READY.value
        response.message = "lifecycle node is not active"
        return False


def _worker_timeout(device_timeout_s: float) -> float:
    return device_timeout_s + max(0.05, device_timeout_s * 0.1)


def _set_failure(
    response: Any,
    error: BaseException,
    *,
    accepted_field: bool = False,
) -> None:
    if accepted_field:
        response.accepted = False
    else:
        response.success = False
    response.error_code = (
        error.code.value if isinstance(error, HandError) else ErrorCode.INTERNAL.value
    )
    response.message = str(error)


__all__ = ["ManagementInterface"]
