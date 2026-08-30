"""Bounded periodic callback worker for cache-only ROS publication."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable


class PeriodicCallbackWorker:
    """Run one callback at a replaceable rate without building a tick backlog."""

    def __init__(self, callback: Callable[[], None], *, name: str) -> None:
        if not callable(callback):
            raise TypeError("callback must be callable")
        if not isinstance(name, str) or not name:
            raise ValueError("name must be a non-empty string")
        self._callback = callback
        self._name = name
        self._condition = threading.Condition()
        self._thread: threading.Thread | None = None
        self._stop_requested = False
        self._rate_hz = 0.0
        self._revision = 0

    def start(self, rate_hz: float) -> None:
        """Start once or replace the rate of an already running worker."""

        rate = _rate(rate_hz)
        with self._condition:
            thread = self._thread
            if thread is not None and thread.is_alive():
                self._replace_rate_locked(rate)
                return
            self._stop_requested = False
            self._rate_hz = rate
            self._revision += 1
            self._thread = threading.Thread(
                target=self._run,
                name=self._name,
                daemon=False,
            )
            self._thread.start()

    def set_rate(self, rate_hz: float) -> None:
        """Replace the active rate; zero pauses without ending the thread."""

        rate = _rate(rate_hz)
        with self._condition:
            self._replace_rate_locked(rate)

    def stop(self, timeout_s: float) -> None:
        """Stop idempotently and wait boundedly for the callback to return."""

        timeout = _timeout(timeout_s)
        with self._condition:
            thread = self._thread
            if thread is None:
                return
            self._stop_requested = True
            self._condition.notify_all()
        thread.join(timeout)
        if thread.is_alive():
            raise RuntimeError(f"periodic worker {self._name} did not stop in time")
        with self._condition:
            if self._thread is thread:
                self._thread = None

    @property
    def active(self) -> bool:
        with self._condition:
            return self._thread is not None and self._thread.is_alive()

    @property
    def rate_hz(self) -> float:
        with self._condition:
            return self._rate_hz

    def _replace_rate_locked(self, rate_hz: float) -> None:
        self._rate_hz = rate_hz
        self._revision += 1
        self._condition.notify_all()

    def _run(self) -> None:
        revision = -1
        next_deadline = time.monotonic()
        while True:
            with self._condition:
                while not self._stop_requested and self._rate_hz == 0:
                    self._condition.wait()
                if self._stop_requested:
                    return
                if revision != self._revision:
                    revision = self._revision
                    next_deadline = time.monotonic()
                rate_hz = self._rate_hz
                period_s = 1.0 / rate_hz
                remaining = next_deadline - time.monotonic()
                if remaining > 0:
                    self._condition.wait(remaining)
                    continue

            self._callback()
            next_deadline += period_s
            now = time.monotonic()
            if next_deadline <= now:
                skipped = math.floor((now - next_deadline) / period_s) + 1
                next_deadline += skipped * period_s


def _rate(value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0
    ):
        raise ValueError("rate_hz must be finite and non-negative")
    return float(value)


def _timeout(value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0
    ):
        raise ValueError("timeout_s must be finite and non-negative")
    return float(value)


__all__ = ["PeriodicCallbackWorker"]
