from dataclasses import replace

import pytest

from linkerbot_hand_ros.benchmark import (
    BenchmarkMeasurements,
    DiagnosticObservation,
    StateEvent,
    evaluate_measurements,
)
from linkerbot_hand_ros.examples.frequency_benchmark import (
    _parse_args,
    _secondary_sensor,
    _validate_args,
)


def _diagnostic(
    observed_at,
    *,
    angle_sequence,
    torque_sequence,
    generation="1",
    accepted=0,
    superseded=0,
    rejected=0,
    feedback_errors=0,
    pending=0,
    angle_stage="dispatched",
):
    return DiagnosticObservation(
        observed_at=observed_at,
        values={
            "connected": "True",
            "connection_generation": generation,
            "consecutive_errors": "0",
            "pending_commands": str(pending),
            "superseded_commands": str(superseded),
            "ros_commands_accepted": str(accepted),
            "ros_commands_rejected": str(rejected),
            "ros_accepted_feedback_errors": str(feedback_errors),
            "latest_angles_stage": angle_stage,
            "angles_sequence": str(angle_sequence),
            "angles_connection_generation": generation,
            "torques_sequence": str(torque_sequence),
            "torques_connection_generation": generation,
        },
    )


def _measurements(*, final_angle_sequence=500, final_torque_sequence=500):
    sent_at = tuple(index / 50.0 for index in range(500))
    accepted = tuple(("angles", value) for value in sent_at)
    states = tuple(
        StateEvent(sensor, index / 50.0, True)
        for index in range(500)
        for sensor in ("angles", "torques")
    )
    start = _diagnostic(
        0.0,
        angle_sequence=0,
        torque_sequence=0,
    )
    middle = _diagnostic(
        5.0,
        angle_sequence=final_angle_sequence // 2,
        torque_sequence=final_torque_sequence // 2,
        accepted=250,
    )
    end = _diagnostic(
        10.0,
        angle_sequence=final_angle_sequence,
        torque_sequence=final_torque_sequence,
        accepted=500,
    )
    return BenchmarkMeasurements(
        requested_hz=50.0,
        started_at=0.0,
        ended_at=10.0,
        command_names=("angles",),
        required_sensors=("angles", "torques"),
        sent_at=sent_at,
        accepted_commands=accepted,
        state_events=states,
        diagnostic_samples=(start, middle, end),
        start_diagnostic=start,
        end_diagnostic=end,
    )


def test_benchmark_passes_only_when_commands_and_real_samples_meet_target():
    result = evaluate_measurements(_measurements())

    assert result.passed
    assert result.send_hz == pytest.approx(50.0)
    assert result.send_interval_p95_ms == pytest.approx(20.0)
    assert dict(result.state_hz) == {"angles": 50.0, "torques": 50.0}
    assert dict(result.sample_hz) == {"angles": 50.0, "torques": 50.0}


def test_repeated_ros_cache_publications_do_not_hide_slow_device_sampling():
    result = evaluate_measurements(
        _measurements(final_angle_sequence=300, final_torque_sequence=300)
    )

    assert not result.passed
    assert dict(result.state_hz) == {"angles": 50.0, "torques": 50.0}
    assert dict(result.sample_hz) == {"angles": 30.0, "torques": 30.0}
    assert any("设备采样频率" in reason for reason in result.reasons)


def test_benchmark_rejects_connection_change_and_nonterminal_command():
    measurements = _measurements()
    changed_end = _diagnostic(
        10.0,
        angle_sequence=500,
        torque_sequence=500,
        generation="2",
        angle_stage="superseded",
    )
    measurements = replace(measurements, end_diagnostic=changed_end)

    result = evaluate_measurements(measurements)

    assert not result.passed
    assert "测试期间连接代次发生变化" in result.reasons
    assert "angles 最新命令终态为 superseded" in result.reasons


def test_missing_health_counter_cannot_be_reported_as_success():
    measurements = _measurements()
    values = dict(measurements.end_diagnostic.values)
    values.pop("pending_commands")
    values.pop("ros_commands_rejected")
    end = DiagnosticObservation(10.0, values)

    result = evaluate_measurements(replace(measurements, end_diagnostic=end))

    assert not result.passed
    assert "诊断中缺少待发送命令数" in result.reasons
    assert "诊断中缺少 ros_commands_rejected" in result.reasons


def test_control_mode_requires_explicit_torque_limit():
    parsed = _parse_args([])

    with pytest.raises(ValueError, match="必须显式提供"):
        _validate_args(parsed)


def test_default_candidates_are_50_through_100_hz():
    parsed = _parse_args(["--mode", "angles"])

    _validate_args(parsed)

    assert parsed.rates == [50.0, 60.0, 80.0, 100.0]
    assert parsed.repetitions == 3
    assert parsed.duration == 30.0
    assert parsed.feedback == "angles-and-load"


@pytest.mark.parametrize(
    ("model", "expected"),
    (("L6", "torques"), ("O6", "torques"), ("L30", "current"), ("O20", "current")),
)
def test_secondary_feedback_uses_current_only_without_torque_sensor(model, expected):
    assert _secondary_sensor(model) == expected
