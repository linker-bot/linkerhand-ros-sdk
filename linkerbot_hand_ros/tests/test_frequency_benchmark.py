from copy import deepcopy
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

from linkerbot_hand_msgs.msg import (
    HandCommandStatus,
    HandCommandStatusBatch,
    HandCommandTransition,
)

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
    angle_completed=0,
    angle_command_feedback=0,
    angle_feedback=None,
    feedback_policy="query_only",
    force_completed=0,
):
    angle_feedback = angle_completed if angle_feedback is None else angle_feedback
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
            "channel_angles_completed_reads": str(angle_completed),
            "channel_angles_command_feedback_samples": str(angle_command_feedback),
            "channel_angles_feedback_samples": str(angle_feedback),
            "channel_angles_feedback_policy": feedback_policy,
            "channel_force_completed_reads": str(force_completed),
        },
    )


def _state_events(channel, *, cache_updates, publications):
    return tuple(
        StateEvent(
            channel,
            index * 10 / publications,
            round(index * cache_updates / publications),
            1,
            True,
            False,
        )
        for index in range(publications + 1)
    )


def _measurements(
    *,
    angle_sequence=500,
    angle_publications=None,
    angle_reads=500,
    angle_command_feedback=0,
    angle_feedback=None,
    feedback_policy="query_only",
    tactile=False,
):
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
    angle_publications = (
        angle_sequence if angle_publications is None else angle_publications
    )
    states = list(
        _state_events(
            "angles",
            cache_updates=angle_sequence,
            publications=angle_publications,
        )
    )
    if tactile:
        states.extend(_state_events("tactile", cache_updates=300, publications=300))
    end_diagnostic = _diagnostic(
        10,
        angle_completed=angle_reads,
        angle_command_feedback=angle_command_feedback,
        angle_feedback=(angle_reads if angle_feedback is None else angle_feedback),
        feedback_policy=feedback_policy,
        force_completed=300 if tactile else 0,
    )
    return BenchmarkMeasurements(
        client_id=_CLIENT_ID,
        requested_angle_hz=50,
        requested_tactile_hz=30 if tactile else 0,
        angle_feedback_policy=feedback_policy,
        started_at=0,
        ended_at=10,
        sent_at=sent,
        state_events=tuple(states),
        command_events=commands,
        start_diagnostic=_diagnostic(0, feedback_policy=feedback_policy),
        measurement_end_diagnostic=end_diagnostic,
        end_diagnostic=end_diagnostic,
    )


def test_equal_workload_passes_at_requested_angle_rate():
    result = evaluate_measurements(_measurements())

    assert result.passed
    assert result.command_publish_hz == 50
    assert result.command_received_hz == 50
    assert result.command_accepted_hz == 50
    assert result.command_terminal_hz == 50
    assert result.command_dispatch_hz == 50
    assert result.angle_feedback_hz == 50
    assert result.angle_command_feedback_hz == 0
    assert result.angle_read_completion_hz == 50
    assert result.angle_cache_update_hz == 50
    assert result.angle_ros_publish_hz == 50
    assert result.tactile_read_completion_hz is None
    assert result.publish_to_receive_latency.sample_count == 500
    assert result.publish_to_receive_latency.p95_ms == pytest.approx(1.0)
    assert result.receive_to_accept_latency.p95_ms == pytest.approx(0.5)
    assert result.accept_to_dispatch_latency.p95_ms == pytest.approx(1.5)
    assert result.publish_to_dispatch_latency.p95_ms == pytest.approx(3.0)


def test_read_completion_rate_is_not_inflated_by_device_reports():
    result = evaluate_measurements(
        _measurements(
            angle_sequence=1_000,
            angle_publications=500,
            angle_reads=300,
        )
    )

    assert not result.passed
    assert result.angle_read_completion_hz == 30
    assert result.angle_cache_update_hz == 100
    assert result.angle_ros_publish_hz == 50
    assert any("角度主动查询完成频率" in reason for reason in result.reasons)


def test_command_feedback_policy_uses_verified_echoes_instead_of_active_reads():
    result = evaluate_measurements(
        _measurements(
            angle_reads=0,
            angle_command_feedback=500,
            angle_feedback=500,
            feedback_policy="command_feedback_only",
        )
    )

    assert result.passed
    assert result.angle_read_completion_hz == 0
    assert result.angle_command_feedback_hz == 50
    assert result.angle_feedback_hz == 50


def test_adaptive_policy_accepts_combined_query_and_command_feedback() -> None:
    result = evaluate_measurements(
        _measurements(
            angle_reads=200,
            angle_command_feedback=300,
            angle_feedback=500,
            feedback_policy="adaptive",
        )
    )

    assert result.passed
    assert result.angle_read_completion_hz == 20
    assert result.angle_command_feedback_hz == 30
    assert result.angle_feedback_hz == 50


def test_low_cache_and_ros_rates_are_reported_separately():
    result = evaluate_measurements(_measurements(angle_sequence=300))

    assert not result.passed
    assert result.angle_read_completion_hz == 50
    assert result.angle_cache_update_hz == 30
    assert result.angle_ros_publish_hz == 30


def test_ros_publication_loss_cannot_be_hidden_by_cache_sequence_jumps():
    result = evaluate_measurements(
        _measurements(angle_sequence=500, angle_publications=300)
    )

    assert not result.passed
    assert result.angle_read_completion_hz == 50
    assert result.angle_cache_update_hz == 50
    assert result.angle_ros_publish_hz == 30
    assert any("角度 ROS 发布频率" in reason for reason in result.reasons)


def test_complete_tactile_rate_is_evaluated_independently():
    result = evaluate_measurements(_measurements(tactile=True))

    assert result.passed
    assert result.tactile_read_completion_hz == 30
    assert result.tactile_cache_update_hz == 30
    assert result.tactile_ros_publish_hz == 30


def test_drain_reads_do_not_inflate_measurement_rate():
    measurements = _measurements()
    drained = _diagnostic(11, angle_completed=600)

    result = evaluate_measurements(replace(measurements, end_diagnostic=drained))

    assert result.passed
    assert result.angle_read_completion_hz == 50


def test_polling_counter_reset_cannot_be_reported_as_a_read_rate():
    measurements = _measurements()
    start = _diagnostic(0, angle_completed=100)
    measurement_end = _diagnostic(10, angle_completed=50)

    result = evaluate_measurements(
        replace(
            measurements,
            start_diagnostic=start,
            measurement_end_diagnostic=measurement_end,
            end_diagnostic=measurement_end,
        )
    )

    assert not result.passed
    assert result.angle_read_completion_hz is None
    assert any("角度有效反馈频率 unknown" in reason for reason in result.reasons)
    assert any("角度主动查询完成频率 unknown" in reason for reason in result.reasons)


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

        foreign = deepcopy(status)
        foreign.client_id = client_id + 1
        node._on_command_status_batch(
            HandCommandStatusBatch(statuses=[foreign, status])
        )

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
    measurement_end = _diagnostic(
        10,
        generation="2",
        timeouts=1,
        angle_completed=500,
    )
    end = _diagnostic(
        11,
        generation="2",
        timeouts=1,
        angle_completed=550,
    )

    result = evaluate_measurements(
        replace(
            measurements,
            state_events=tuple(states),
            measurement_end_diagnostic=measurement_end,
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
