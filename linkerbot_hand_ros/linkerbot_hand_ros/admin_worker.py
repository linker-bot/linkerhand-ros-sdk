"""Bounded worker for low-frequency synchronous SDK management reads."""

# Worker boundaries must preserve arbitrary SDK/backend exceptions in Futures.
# ruff: noqa: BLE001

from __future__ import annotations

import math
import queue
import threading
from collections.abc import Callable
from concurrent.futures import Future
from concurrent.futures import TimeoutError as FutureTimeoutError
from dataclasses import dataclass
from typing import Generic, TypeVar

from linkerbot_hand import (
    ClosedError,
    LifecycleError,
    QueueFullError,
    TimeoutError,
    ValidationError,
)

_ResultT = TypeVar("_ResultT")


@dataclass(slots=True)
class _Task(Generic[_ResultT]):
    operation: str
    callback: Callable[[], _ResultT]
    future: Future[_ResultT]


_STOP = object()


class AdminWorker:
    """Run active SDK getters outside ROS executor callback threads."""

    def __init__(self, *, capacity: int = 16, name: str = "hand-admin") -> None:
        if type(capacity) is not int or capacity <= 0:
            raise ValidationError("admin worker capacity must be a positive int")
        self._queue: queue.Queue[_Task[object] | object] = queue.Queue(capacity)
        self._name = name
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._accepting = False

    def start(self) -> None:
        """Start the worker once; repeated active starts are no-ops."""

        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._accepting = True
            thread = threading.Thread(target=self._run, name=self._name, daemon=False)
            self._thread = thread
            thread.start()

    def call(
        self,
        operation: str,
        callback: Callable[[], _ResultT],
        *,
        timeout_s: float,
    ) -> _ResultT:
        """Enqueue one bounded task and wait only in the management callback."""

        timeout = _positive_timeout(timeout_s)
        if not isinstance(operation, str) or not operation:
            raise ValidationError("admin operation must be a non-empty string")
        if not callable(callback):
            raise ValidationError("admin callback must be callable")
        future: Future[_ResultT] = Future()
        with self._lock:
            if not self._accepting:
                raise ClosedError(
                    "admin worker is not active",
                    operation=operation,
                )
            try:
                self._queue.put_nowait(_Task(operation, callback, future))
            except queue.Full as error:
                raise QueueFullError(
                    "admin worker queue is full",
                    operation=operation,
                    cause=error,
                ) from error
        try:
            return future.result(timeout=timeout)
        except FutureTimeoutError as error:
            future.cancel()
            raise TimeoutError(
                f"{operation} did not finish within {timeout:.3f}s",
                operation=operation,
                cause=error,
            ) from error

    def stop(self, *, timeout_s: float) -> None:
        """Reject new work, cancel queued tasks, and join with a finite bound."""

        timeout = _positive_timeout(timeout_s)
        with self._lock:
            thread = self._thread
            if thread is None:
                self._accepting = False
                return
            if not thread.is_alive():
                self._accepting = False
                self._cancel_queued()
                self._thread = None
                return
            self._accepting = False
            self._cancel_queued()
            self._queue.put_nowait(_STOP)
        thread.join(timeout)
        if thread.is_alive():
            raise LifecycleError(
                f"admin worker did not stop within {timeout:.3f}s",
                operation="stop_admin_worker",
            )
        with self._lock:
            if self._thread is thread:
                self._thread = None

    @property
    def active(self) -> bool:
        with self._lock:
            return self._accepting

    @property
    def pending(self) -> int:
        return self._queue.qsize()

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is _STOP:
                    return
                assert isinstance(item, _Task)
                if item.future.set_running_or_notify_cancel():
                    try:
                        result = item.callback()
                    except Exception as error:
                        item.future.set_exception(error)
                    else:
                        item.future.set_result(result)
            finally:
                self._queue.task_done()

    def _cancel_queued(self) -> None:
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                return
            try:
                if isinstance(item, _Task) and not item.future.done():
                    item.future.set_exception(
                        ClosedError(
                            "admin worker stopped before dispatch",
                            operation=item.operation,
                        )
                    )
            finally:
                self._queue.task_done()


def _positive_timeout(value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValidationError("timeout_s must be finite and positive")
    return float(value)


__all__ = ["AdminWorker"]
