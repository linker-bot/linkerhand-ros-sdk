import threading
import time

import pytest
from linkerbot_hand_ros.periodic_worker import PeriodicCallbackWorker


def test_periodic_worker_reconfigures_pauses_and_stops_boundedly():
    calls: list[float] = []
    worker = PeriodicCallbackWorker(
        lambda: calls.append(time.monotonic()),
        name="periodic-test",
    )
    worker.start(100.0)
    try:
        deadline = time.monotonic() + 0.2
        while len(calls) < 3 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert len(calls) >= 3

        worker.set_rate(0.0)
        paused_count = len(calls)
        time.sleep(0.03)
        assert len(calls) == paused_count

        worker.set_rate(200.0)
        deadline = time.monotonic() + 0.2
        while len(calls) == paused_count and time.monotonic() < deadline:
            time.sleep(0.005)
        assert len(calls) > paused_count
    finally:
        worker.stop(0.5)

    assert not worker.active
    stopped_count = len(calls)
    time.sleep(0.02)
    assert len(calls) == stopped_count
    worker.stop(0.5)


def test_periodic_worker_stop_reports_an_inflight_callback_timeout():
    entered = threading.Event()
    release = threading.Event()
    worker = PeriodicCallbackWorker(
        lambda: (entered.set(), release.wait(1.0)),
        name="periodic-blocked-test",
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
def test_periodic_worker_rejects_invalid_rates(value):
    worker = PeriodicCallbackWorker(lambda: None, name="periodic-invalid-test")

    with pytest.raises(ValueError, match="rate_hz"):
        worker.start(value)
