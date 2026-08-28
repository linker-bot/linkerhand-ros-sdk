from linkerbot_hand_msgs.msg import (
    HandCommandStatus,
    HandJointCommand,
    HandJointState,
    HandTelemetry,
    SampleMetadata,
)
from linkerbot_hand_msgs.srv import Recover, SetRawValues, SubmitCommand


def test_joint_command_requires_an_explicit_value_representation():
    command = HandJointCommand(
        user_sequence=7,
        representation=HandJointCommand.REPRESENTATION_PERCENT,
        values=[100.0, 50.0],
    )

    assert command.representation == HandJointCommand.REPRESENTATION_PERCENT
    assert HandJointCommand.REPRESENTATION_RAW != command.representation
    assert command.user_sequence == 7


def test_joint_state_keeps_raw_percent_and_physical_validity_separate():
    state = HandJointState(
        joint_names=["thumb_flex", "thumb_abd"],
        raw=[255.0, 128.0],
        percent=[100.0, 50.2],
        percent_valid=[True, True],
        physical_rad=[0.0, 0.0],
        physical_rad_valid=[False, False],
        meta=SampleMetadata(
            sequence=3,
            age_s=0.002,
            valid=True,
            stale=False,
            connection_generation=2,
            origin=SampleMetadata.ORIGIN_DEVICE_RESPONSE,
            within_declared_range=True,
        ),
    )

    assert list(state.raw) == [255.0, 128.0]
    assert list(state.percent) == [100.0, 50.2]
    assert not any(state.physical_rad_valid)
    assert state.meta.connection_generation == 2


def test_telemetry_declares_configuration_and_measurement_origin():
    configured = HandTelemetry(
        semantic="configured_speeds",
        channel_names=["thumb_flex"],
        raw=[128.0],
        raw_unit="count",
        device_measured=False,
        device_queried=False,
    )

    assert configured.semantic == "configured_speeds"
    assert configured.device_measured is False
    assert configured.device_queried is False


def test_command_status_does_not_conflate_dispatch_with_acknowledgement():
    status = HandCommandStatus(
        command_id=11,
        family="angle_target",
        stage=HandCommandStatus.STAGE_DISPATCHED,
        ack_support=HandCommandStatus.ACK_UNSUPPORTED,
        sent_frames=1,
    )

    assert status.stage == HandCommandStatus.STAGE_DISPATCHED
    assert status.stage != HandCommandStatus.STAGE_ACKNOWLEDGED
    assert status.ack_support == HandCommandStatus.ACK_UNSUPPORTED


def test_admin_services_return_structured_acceptance_and_errors():
    submit = SubmitCommand.Response(accepted=True, command_id=5)
    set_values = SetRawValues.Request(user_sequence=9, raw=[1.0, 2.0])
    recover = Recover.Response(success=False, error_code="timeout")

    assert submit.accepted and submit.command_id == 5
    assert set_values.user_sequence == 9
    assert recover.error_code == "timeout"
