from dataclasses import replace

import rclpy
from linkerbot_hand_ros.benchmark import (
    BenchmarkMeasurements,
    CommandEvent,
    DiagnosticObservation,
    StateEvent,
    evaluate_measurements,
)
from linkerbot_hand_ros.examples.frequency_benchmark import FrequencyBenchmarkNode

from linkerbot_hand_msgs.msg import HandCommandStatus


def test_benchmark_defers_high_rate_observation_until_capability_discovery():
    rclpy.init()
    node = FrequencyBenchmarkNode("/linkerbot_hand")
    try:
        client_subscriptions = tuple(node.command_client._subscriptions)
        assert not node._observation_subscriptions
        assert all(
            subscription in node.subscriptions for subscription in client_subscriptions
        )

        node.start_observing()
        subscriptions = tuple(node._observation_subscriptions)
        node.start_observing()

        assert len(subscriptions) == 4
        assert tuple(node._observation_subscriptions) == subscriptions
        assert all(
            subscription in node.subscriptions
            for subscription in (*client_subscriptions, *subscriptions)
        )
    finally:
        node.destroy_benchmark()
        node.destroy_node()
        rclpy.try_shutdown()


def _diagnostic(
    observed_at,
    *,
    generation="1",
    pending=0,
    timeouts=0,
    consecutive_errors=0,
):
    return DiagnosticObservation(
        observed_at,
        {
            "connection_generation": generation,
            "pending_commands": str(pending),
            "consecutive_errors": str(consecutive_errors),
            "transport_dropped_frames": "0",
            "transport_route_misses": "0",
            "transport_bus_errors": "0",
            "channel_angles_timeouts": str(timeouts),
            "channel_force_timeouts": "0",
        },
    )


def _measurements(*, angle_sequence=500, tactile=False):
    sent = tuple(index / 50 for index in range(500))
    commands = tuple(
        CommandEvent(
            observed_at=value,
            user_sequence=index + 1,
            stage=HandCommandStatus.STAGE_DISPATCHED,
            ack_support=HandCommandStatus.ACK_UNSUPPORTED,
        )
        for index, value in enumerate(sent)
    )
    states = [
        StateEvent(
            "angles",
            index * 10 / angle_sequence,
            index,
            1,
            True,
            False,
        )
        for index in range(angle_sequence + 1)
    ]
    if tactile:
        states.extend(
            StateEvent("tactile", index * 10 / 300, index, 1, True, False)
            for index in range(301)
        )
    return BenchmarkMeasurements(
        requested_angle_hz=50,
        requested_tactile_hz=30 if tactile else 0,
        started_at=0,
        ended_at=10,
        sent_at=sent,
        state_events=tuple(states),
        command_events=commands,
        start_diagnostic=_diagnostic(0),
        end_diagnostic=_diagnostic(10),
    )


def test_equal_workload_passes_at_requested_angle_rate():
    result = evaluate_measurements(_measurements())

    assert result.passed
    assert result.command_publish_hz == 50
    assert result.command_terminal_hz == 50
    assert result.angle_sample_hz == 50


def test_repeated_ros_publication_cannot_hide_low_device_sequence_rate():
    result = evaluate_measurements(_measurements(angle_sequence=300))

    assert not result.passed
    assert result.angle_sample_hz == 30


def test_complete_tactile_rate_is_evaluated_independently():
    result = evaluate_measurements(_measurements(tactile=True))

    assert result.passed
    assert result.tactile_sample_hz == 30


def test_failed_or_superseded_commands_fail_the_candidate():
    measurements = _measurements()
    events = list(measurements.command_events)
    events[0] = replace(events[0], stage=HandCommandStatus.STAGE_FAILED)
    events[1] = replace(events[1], stage=HandCommandStatus.STAGE_SUPERSEDED)

    result = evaluate_measurements(replace(measurements, command_events=tuple(events)))

    assert not result.passed
    assert result.failed_commands == 1
    assert result.superseded_commands == 1


def test_generation_change_and_health_timeout_fail_the_candidate():
    measurements = _measurements()
    states = list(measurements.state_events)
    states[-1] = replace(states[-1], generation=2)
    end = _diagnostic(10, generation="2", timeouts=1)

    result = evaluate_measurements(
        replace(
            measurements,
            state_events=tuple(states),
            end_diagnostic=end,
        )
    )

    assert not result.passed
    assert any("连接代次" in reason for reason in result.reasons)
    assert any("角度事务超时" in reason for reason in result.reasons)


def test_invalid_sample_still_exposes_connection_generation_change():
    measurements = _measurements()
    states = list(measurements.state_events)
    states.append(replace(states[-1], generation=2, valid=False))

    result = evaluate_measurements(replace(measurements, state_events=tuple(states)))

    assert not result.passed
    assert any("连接代次" in reason for reason in result.reasons)


def test_nonzero_consecutive_error_gauge_fails_even_when_start_was_higher():
    measurements = _measurements()

    result = evaluate_measurements(
        replace(
            measurements,
            start_diagnostic=_diagnostic(0, consecutive_errors=2),
            end_diagnostic=_diagnostic(10, consecutive_errors=1),
        )
    )

    assert not result.passed
    assert any("连续 SDK 错误当前为 1" in reason for reason in result.reasons)
