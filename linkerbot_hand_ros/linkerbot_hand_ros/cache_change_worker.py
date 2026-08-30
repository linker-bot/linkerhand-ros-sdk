"""Bounded worker for change-driven SDK cache consumption."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable

_MAX_WAIT_S = 0.05


class CacheChangeWorker:
    """Publish cache changes immediately and refresh idle state periodically."""

    def __init__(
        self,
        wait_for_change: Callable[[float], bool],
        refresh_idle: Callable[[], None],
        *,
        name: str,
    ) -> None:
        if not callable(wait_for_change):
            raise TypeError("wait_for_change must be callable")
        if not callable(refresh_idle):
            raise TypeError("refresh_idle must be callable")
        if not isinstance(name, str) or not name:
            raise ValueError("name must be a non-empty string")
        self._wait_for_change = wait_for_change
        self._refresh_idle = refresh_idle
        self._name = name
        self._condition = threading.Condition()
        self._thread: threading.Thread | None = None
        self._stop_requested = False
        self._rate_hz = 0.0
        self._rate_revision = 0

    def start(self, rate_hz: float) -> None:
        """Start once or replace the idle cache-check rate."""

        rate = _rate(rate_hz)
        with self._condition:
            thread = self._thread
            if thread is not None and thread.is_alive():
                self._replace_rate_locked(rate)
                return
            self._stop_requested = False
            self._rate_hz = rate
            self._rate_revision += 1
            self._thread = threading.Thread(
                target=self._run,
                name=self._name,
                daemon=False,
            )
            self._thread.start()

    def set_rate(self, rate_hz: float) -> None:
        """Replace the idle cache-check rate; zero pauses the worker."""

        rate = _rate(rate_hz)
        with self._condition:
            self._replace_rate_locked(rate)

    def stop(self, timeout_s: float) -> None:
        """Stop idempotently and wait boundedly for the cache wait to return."""

        timeout = _timeout(timeout_s)
        with self._condition:
            thread = self._thread
            if thread is None:
                return
            self._stop_requested = True
            self._condition.notify_all()
        thread.join(timeout)
        if thread.is_alive():
            raise RuntimeError(f"cache change worker {self._name} did not stop in time")
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
        self._rate_revision += 1
        self._condition.notify_all()

    def _run(self) -> None:
        next_refresh_at: float | None = None
        active_revision = -1
        while True:
            with self._condition:
                while not self._stop_requested and self._rate_hz == 0:
                    next_refresh_at = None
                    self._condition.wait()
                if self._stop_requested:
                    return
                rate_hz = self._rate_hz
                rate_revision = self._rate_revision
                if rate_revision != active_revision:
                    next_refresh_at = time.monotonic() + 1.0 / rate_hz
                    active_revision = rate_revision
            period_s = 1.0 / rate_hz
            if next_refresh_at is None:  # pragma: no cover - initialized above
                next_refresh_at = time.monotonic() + period_s
            remaining = next_refresh_at - time.monotonic()
            if remaining <= 0:
                self._refresh_idle()
                completed_at = time.monotonic()
                periods = (
                    math.floor(max(0.0, completed_at - next_refresh_at) / period_s) + 1
                )
                next_refresh_at += periods * period_s
                continue
            changed = self._wait_for_change(min(_MAX_WAIT_S, remaining))
            completed_at = time.monotonic()
            if changed:
                next_refresh_at = completed_at + period_s
                continue
            if completed_at + 1e-12 < next_refresh_at:
                continue
            self._refresh_idle()
            periods = (
                math.floor(max(0.0, completed_at - next_refresh_at) / period_s) + 1
            )
            next_refresh_at += periods * period_s


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


__all__ = ["CacheChangeWorker"]
