"""Thread-safe runtime metrics for driver diagnostics."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class DriverMetricsSnapshot:
    uptime_sec: float
    state_successes: int
    state_errors: int
    command_successes: int
    command_errors: int
    feedback_errors: int
    last_state_age_sec: float | None
    last_command_age_sec: float | None
    latest_state_failed: bool
    latest_command_failed: bool
    latest_feedback_failed: bool
    last_error: str


class DriverMetrics:
    """Collect counters and event freshness across executor threads."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._started_at = time.monotonic()
        self._state_successes = 0
        self._state_errors = 0
        self._command_successes = 0
        self._command_errors = 0
        self._feedback_errors = 0
        self._last_state_success_at: float | None = None
        self._last_command_at: float | None = None
        self._latest_state_failed = False
        self._latest_command_failed = False
        self._latest_feedback_failed = False
        self._last_error = ""

    def record_state_success(self) -> None:
        with self._lock:
            self._state_successes += 1
            self._last_state_success_at = time.monotonic()
            self._latest_state_failed = False

    def record_state_error(self, error: Exception) -> None:
        with self._lock:
            self._state_errors += 1
            self._latest_state_failed = True
            self._last_error = str(error)

    def record_command_success(self) -> None:
        with self._lock:
            self._command_successes += 1
            self._last_command_at = time.monotonic()
            self._latest_command_failed = False

    def record_command_error(self, error: Exception) -> None:
        with self._lock:
            self._command_errors += 1
            self._last_command_at = time.monotonic()
            self._latest_command_failed = True
            self._last_error = str(error)

    def record_feedback_success(self) -> None:
        with self._lock:
            self._latest_feedback_failed = False

    def record_feedback_error(self, error: Exception) -> None:
        with self._lock:
            self._feedback_errors += 1
            self._latest_feedback_failed = True
            self._last_error = str(error)

    def snapshot(self) -> DriverMetricsSnapshot:
        now = time.monotonic()
        with self._lock:
            return DriverMetricsSnapshot(
                uptime_sec=now - self._started_at,
                state_successes=self._state_successes,
                state_errors=self._state_errors,
                command_successes=self._command_successes,
                command_errors=self._command_errors,
                feedback_errors=self._feedback_errors,
                last_state_age_sec=(
                    None
                    if self._last_state_success_at is None
                    else now - self._last_state_success_at
                ),
                last_command_age_sec=(
                    None
                    if self._last_command_at is None
                    else now - self._last_command_at
                ),
                latest_state_failed=self._latest_state_failed,
                latest_command_failed=self._latest_command_failed,
                latest_feedback_failed=self._latest_feedback_failed,
                last_error=self._last_error,
            )
