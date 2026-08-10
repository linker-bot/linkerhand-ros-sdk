import pytest

from linkerbot_hand_ros import metrics
from linkerbot_hand_ros.metrics import DriverMetrics


def test_metrics_track_freshness_counters_and_latest_outcome(monkeypatch):
    timestamps = iter((10.0, 12.0, 13.0, 14.0, 15.0, 16.0))
    monkeypatch.setattr(metrics.time, "monotonic", lambda: next(timestamps))
    driver_metrics = DriverMetrics()

    driver_metrics.record_state_success()
    driver_metrics.record_command_success()
    healthy = driver_metrics.snapshot()

    assert healthy.uptime_sec == 4.0
    assert healthy.last_state_age_sec == 2.0
    assert healthy.last_command_age_sec == 1.0
    assert not healthy.latest_state_failed
    assert not healthy.latest_command_failed

    driver_metrics.record_state_error(RuntimeError("read timeout"))
    failed = driver_metrics.snapshot()

    assert failed.state_successes == 1
    assert failed.state_errors == 1
    assert failed.latest_state_failed
    assert failed.last_error == "read timeout"


def test_metrics_report_never_before_first_activity(monkeypatch):
    timestamps = iter((20.0, 21.5))
    monkeypatch.setattr(metrics.time, "monotonic", lambda: next(timestamps))

    snapshot = DriverMetrics().snapshot()

    assert snapshot.uptime_sec == pytest.approx(1.5)
    assert snapshot.last_state_age_sec is None
    assert snapshot.last_command_age_sec is None
