from dataclasses import replace

import pytest
import rclpy
from builtin_interfaces.msg import Time
from linkerbot_hand_ros.benchmark import (
    BenchmarkMeasurements,
    CommandEvent,
    DiagnosticObservation,
    StateEvent,
    evaluate_measurements,
)
from linkerbot_hand_ros.examples.frequency_benchmark import FrequencyBenchmarkNode

from linkerbot_hand_msgs.msg import HandCommandStatus, HandCommandTransition

_CLIENT_ID = 17


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
            observed_at=value + 0.005,
            client_id=_CLIENT_ID,
            user_sequence=index + 1,
            publisher_at=value,
            driver_received_at=value + 0.001,
            transition_at=value + transition_offset,
            stage=stage,
            ack_support=HandCommandStatus.ACK_UNSUPPORTED,
        )
        for index, value in enumerate(sent)
        for stage, transition_offset in (
            (HandCommandStatus.STAGE_ACCEPTED, 0.0015),
            (HandCommandStatus.STAGE_DISPATCHED, 0.003),
        )
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
        client_id=_CLIENT_ID,
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
    assert result.command_received_hz == 50
    assert result.command_accepted_hz == 50
    assert result.command_terminal_hz == 50
    assert result.command_dispatch_hz == 50
    assert result.angle_sample_hz == 50
    assert result.publish_to_receive_latency.sample_count == 500
    assert result.publish_to_receive_latency.p95_ms == pytest.approx(1.0)
    assert result.receive_to_accept_latency.p95_ms == pytest.approx(0.5)
    assert result.accept_to_dispatch_latency.p95_ms == pytest.approx(1.5)
    assert result.publish_to_dispatch_latency.p95_ms == pytest.approx(3.0)


def test_repeated_ros_publication_cannot_hide_low_device_sequence_rate():
    result = evaluate_measurements(_measurements(angle_sequence=300))

    assert not result.passed
    assert result.angle_sample_hz == 30


def test_complete_tactile_rate_is_evaluated_independently():
    result = evaluate_measurements(_measurements(tactile=True))

    assert result.passed
    assert result.tactile_sample_hz == 30


def test_failed_commands_fail_while_sparse_superseded_targets_are_reported():
    measurements = _measurements()
    events = tuple(
        replace(
            event,
            stage=(
                HandCommandStatus.STAGE_FAILED
                if event.user_sequence == 1
                else HandCommandStatus.STAGE_SUPERSEDED
            ),
        )
        if event.stage == HandCommandStatus.STAGE_DISPATCHED
        and event.user_sequence in {1, 2}
        else event
        for event in measurements.command_events
    )

    result = evaluate_measurements(replace(measurements, command_events=events))

    assert not result.passed
    assert result.failed_commands == 1
    assert result.superseded_commands == 1
    assert result.command_dispatch_hz == 49.8


def test_sparse_latest_wins_replacement_passes_when_dispatch_rate_is_sufficient():
    measurements = _measurements()
    events = tuple(
        replace(event, stage=HandCommandStatus.STAGE_SUPERSEDED)
        if event.stage == HandCommandStatus.STAGE_DISPATCHED
        and event.user_sequence == 1
        else event
        for event in measurements.command_events
    )

    result = evaluate_measurements(replace(measurements, command_events=events))

    assert result.passed
    assert result.superseded_commands == 1
    assert result.command_dispatch_hz == 49.9


def test_replacement_fails_when_actual_dispatch_rate_drops_below_threshold():
    measurements = _measurements()
    events = tuple(
        replace(event, stage=HandCommandStatus.STAGE_SUPERSEDED)
        if event.stage == HandCommandStatus.STAGE_DISPATCHED
        and event.user_sequence <= 51
        else event
        for event in measurements.command_events
    )

    result = evaluate_measurements(replace(measurements, command_events=events))

    assert not result.passed
    assert result.command_terminal_hz == 50
    assert result.command_dispatch_hz == 44.9
    assert any("实际下发频率" in reason for reason in result.reasons)


def test_received_and_accepted_rates_identify_the_loss_boundary():
    measurements = _measurements()
    missing_received = tuple(
        replace(event, driver_received_at=None) if event.user_sequence <= 51 else event
        for event in measurements.command_events
    )

    received_result = evaluate_measurements(
        replace(measurements, command_events=missing_received)
    )

    assert received_result.command_received_hz == 44.9
    assert received_result.command_accepted_hz == 50
    assert any("驱动收到命令频率" in reason for reason in received_result.reasons)

    missing_accepted = tuple(
        event
        for event in measurements.command_events
        if not (
            event.stage == HandCommandStatus.STAGE_ACCEPTED
            and event.user_sequence <= 51
        )
    )
    accepted_result = evaluate_measurements(
        replace(measurements, command_events=missing_accepted)
    )

    assert accepted_result.command_received_hz == 50
    assert accepted_result.command_accepted_hz == 44.9
    assert accepted_result.command_dispatch_hz == 50
    assert any("SDK ACCEPTED 频率" in reason for reason in accepted_result.reasons)


def test_foreign_client_sequence_collision_cannot_inflate_results():
    measurements = _measurements()
    foreign = tuple(
        replace(event, client_id=99) for event in measurements.command_events[:102]
    )

    result = evaluate_measurements(
        replace(
            measurements,
            command_events=(*measurements.command_events, *foreign),
        )
    )

    assert result.passed
    assert result.command_received_hz == 50
    assert result.command_accepted_hz == 50
    assert result.command_dispatch_hz == 50


def test_status_callback_uses_full_history_and_ignores_foreign_client():
    rclpy.init()
    node = FrequencyBenchmarkNode("/linkerbot_hand")
    try:
        client_id = node.command_client.client_id
        status = HandCommandStatus(
            client_id=client_id,
            user_sequence=7,
            publisher_stamp=Time(sec=10),
            driver_received_stamp=Time(sec=10, nanosec=1_000_000),
            stage=HandCommandStatus.STAGE_ACKNOWLEDGED,
            ack_support=HandCommandStatus.ACK_SUPPORTED,
            transitions=[
                HandCommandTransition(
                    stage=HandCommandStatus.STAGE_ACCEPTED,
                    stamp=Time(sec=10, nanosec=2_000_000),
                ),
                HandCommandTransition(
                    stage=HandCommandStatus.STAGE_DISPATCHED,
                    stamp=Time(sec=10, nanosec=3_000_000),
                ),
                HandCommandTransition(
                    stage=HandCommandStatus.STAGE_ACKNOWLEDGED,
                    stamp=Time(sec=10, nanosec=4_000_000),
                ),
            ],
        )

        status.client_id = client_id + 1
        node._on_command_status(status)
        status.client_id = client_id
        node._on_command_status(status)

        assert [event.stage for event in node.command_events] == [
            HandCommandStatus.STAGE_ACCEPTED,
            HandCommandStatus.STAGE_DISPATCHED,
            HandCommandStatus.STAGE_ACKNOWLEDGED,
        ]
        assert all(event.client_id == client_id for event in node.command_events)
    finally:
        node.destroy_benchmark()
        node.destroy_node()
        rclpy.try_shutdown()


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
