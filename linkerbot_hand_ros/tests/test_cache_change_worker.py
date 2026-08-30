import threading
import time
from itertools import pairwise

import pytest
from linkerbot_hand_ros.cache_change_worker import CacheChangeWorker


def test_cache_change_worker_waits_in_bounded_chunks_reconfigures_and_pauses():
    calls: list[float] = []
    called = threading.Event()

    def wait_for_change(timeout_s: float) -> bool:
        calls.append(timeout_s)
        called.set()
        time.sleep(timeout_s)
        return False

    worker = CacheChangeWorker(
        wait_for_change,
        lambda: None,
        name="cache-change-test",
    )
    worker.start(100.0)
    try:
        assert called.wait(0.2)
        assert 0 < calls[0] <= 0.01

        worker.set_rate(0.0)
        time.sleep(0.03)
        paused_count = len(calls)
        time.sleep(0.03)
        assert len(calls) == paused_count

        called.clear()
        worker.set_rate(5.0)
        assert called.wait(0.2)
        assert 0 < calls[-1] <= 0.05
    finally:
        worker.stop(0.5)

    assert not worker.active
    worker.stop(0.5)


def test_cache_change_worker_does_not_rate_limit_immediate_cache_changes():
    observed: list[float] = []
    enough = threading.Event()

    def wait_for_change(timeout_s: float) -> bool:
        assert 0 < timeout_s <= 0.01
        observed.append(time.monotonic())
        if len(observed) >= 4:
            enough.set()
            time.sleep(timeout_s)
            return False
        return True

    worker = CacheChangeWorker(
        wait_for_change,
        lambda: None,
        name="cache-change-immediate-test",
    )
    worker.start(100.0)
    try:
        assert enough.wait(0.2)
    finally:
        worker.stop(0.5)

    assert observed[3] - observed[0] < 0.02


def test_cache_change_worker_does_not_duplicate_refresh_after_fresh_changes():
    refreshed: list[float] = []

    def wait_for_change(timeout_s: float) -> bool:
        time.sleep(min(timeout_s, 0.001))
        return True

    def refresh_age() -> None:
        refreshed.append(time.monotonic())

    worker = CacheChangeWorker(
        wait_for_change,
        refresh_age,
        name="cache-change-continuous-test",
    )
    worker.start(20.0)
    try:
        time.sleep(0.15)
    finally:
        worker.stop(0.5)

    assert refreshed == []


def test_cache_change_worker_refreshes_idle_cache_at_requested_rate():
    refreshed: list[float] = []
    enough = threading.Event()

    def wait_for_change(timeout_s: float) -> bool:
        time.sleep(timeout_s)
        return False

    def refresh_idle() -> None:
        refreshed.append(time.monotonic())
        if len(refreshed) >= 3:
            enough.set()

    worker = CacheChangeWorker(
        wait_for_change,
        refresh_idle,
        name="cache-change-idle-test",
    )
    worker.start(20.0)
    try:
        assert enough.wait(0.3)
    finally:
        worker.stop(0.5)

    intervals = [later - earlier for earlier, later in pairwise(refreshed[:3])]
    assert intervals == pytest.approx([0.05, 0.05], abs=0.015)


def test_cache_change_worker_stop_reports_an_inflight_callback_timeout():
    entered = threading.Event()
    release = threading.Event()
    worker = CacheChangeWorker(
        lambda _timeout: (entered.set(), release.wait(1.0), False)[-1],
        lambda: None,
        name="cache-change-blocked-test",
    )
    worker.start(100.0)
    assert entered.wait(0.5)
    try:
        with pytest.raises(RuntimeError, match="did not stop"):
            worker.stop(0.01)
    finally:
        release.set()
        worker.stop(0.5)


@pytest.mark.parametrize("value", (-1.0, float("inf"), True))
def test_cache_change_worker_rejects_invalid_rates(value):
    worker = CacheChangeWorker(
        lambda _timeout: False,
        lambda: None,
        name="cache-change-invalid",
    )

    with pytest.raises(ValueError, match="rate_hz"):
        worker.start(value)
