"""Publish immutable SDK snapshots without producing device I/O."""

from __future__ import annotations

from linkerbot_hand import (
    DeviceCapabilities,
    HandSnapshot,
    SampleMeta,
    TelemetrySemantic,
)
from rclpy.lifecycle import LifecycleNode

from linkerbot_hand_msgs.msg import (
    HandCapabilities,
    HandDeviceInfo,
    HandJointState,
    HandStallProtectionState,
    HandTactileState,
    HandTelemetry,
)

from .conversion import (
    angle_sample_to_message,
    capabilities_to_message,
    device_info_to_message,
    force_sample_to_messages,
    stall_protection_to_message,
    telemetry_sample_to_message,
)
from .qos import METADATA_QOS, STATE_QOS

_SAMPLE_ATTRIBUTES = {
    TelemetrySemantic.MEASURED_SPEEDS: "measured_speeds",
    TelemetrySemantic.CONFIGURED_SPEEDS: "configured_speeds",
    TelemetrySemantic.CONFIGURED_ACCELERATIONS: "configured_accelerations",
    TelemetrySemantic.CURRENTS: "currents",
    TelemetrySemantic.CURRENT_LIMITS: "current_limits",
    TelemetrySemantic.EFFORT_LIMITS: "effort_limits",
    TelemetrySemantic.TORQUES: "torques",
    TelemetrySemantic.TEMPERATURES: "temperatures",
    TelemetrySemantic.FAULTS: "faults",
    TelemetrySemantic.VOLTAGES: "voltages",
    TelemetrySemantic.MOTION_TIMES: "motion_times",
}
_STALL_ATTRIBUTES = (
    "stall_detection_times",
    "stall_thresholds",
    "stall_hold_currents",
)


class SnapshotPublisher:
    """Own state publishers and suppress duplicate cached samples."""

    def __init__(self, node: LifecycleNode) -> None:
        self._node = node
        plain_publishers = []
        lifecycle_publishers = []
        telemetry_publishers = {}
        try:
            capabilities_publisher = node.create_publisher(
                HandCapabilities,
                "~/capabilities",
                METADATA_QOS,
            )
            plain_publishers.append(capabilities_publisher)
            device_info_publisher = node.create_publisher(
                HandDeviceInfo,
                "~/device_info",
                METADATA_QOS,
            )
            plain_publishers.append(device_info_publisher)
            angles_publisher = node.create_lifecycle_publisher(
                HandJointState,
                "~/state/angles",
                STATE_QOS,
            )
            lifecycle_publishers.append(angles_publisher)
            for semantic in _SAMPLE_ATTRIBUTES:
                publisher = node.create_lifecycle_publisher(
                    HandTelemetry,
                    f"~/state/{semantic.value}",
                    STATE_QOS,
                )
                telemetry_publishers[semantic] = publisher
                lifecycle_publishers.append(publisher)
            force_publisher = node.create_lifecycle_publisher(
                HandTelemetry,
                "~/state/force",
                STATE_QOS,
            )
            lifecycle_publishers.append(force_publisher)
            tactile_publisher = node.create_lifecycle_publisher(
                HandTactileState,
                "~/state/tactile",
                STATE_QOS,
            )
            lifecycle_publishers.append(tactile_publisher)
            stall_publisher = node.create_lifecycle_publisher(
                HandStallProtectionState,
                "~/state/stall_protection",
                STATE_QOS,
            )
            lifecycle_publishers.append(stall_publisher)
        except Exception:
            for publisher in reversed(lifecycle_publishers):
                node.destroy_lifecycle_publisher(publisher)
            for publisher in reversed(plain_publishers):
                node.destroy_publisher(publisher)
            raise
        self._capabilities_publisher = capabilities_publisher
        self._device_info_publisher = device_info_publisher
        self._angles_publisher = angles_publisher
        self._telemetry_publishers = telemetry_publishers
        self._force_publisher = force_publisher
        self._tactile_publisher = tactile_publisher
        self._stall_publisher = stall_publisher
        self._published: dict[str, object] = {}

    def publish_capabilities(self, capabilities: DeviceCapabilities) -> None:
        """Publish declared or verified capability metadata."""

        self._capabilities_publisher.publish(capabilities_to_message(capabilities))

    def publish_snapshot(
        self,
        snapshot: HandSnapshot,
        capabilities: DeviceCapabilities,
        *,
        ros_now_ns: int,
    ) -> int:
        """Publish only samples whose sequence or validity signature changed."""

        publications = 0
        angles = snapshot.angles
        if angles is not None and self._is_changed("angles", angles.meta):
            self._angles_publisher.publish(
                angle_sample_to_message(
                    angles,
                    capabilities,
                    ros_now_ns=ros_now_ns,
                )
            )
            self._mark_published("angles", _metadata_signature(angles.meta))
            publications += 1

        for semantic, attribute in _SAMPLE_ATTRIBUTES.items():
            capability = capabilities.telemetry_channel(semantic)
            sample = getattr(snapshot, attribute)
            if capability is None or sample is None:
                continue
            if not self._is_changed(semantic.value, sample.meta):
                continue
            self._telemetry_publishers[semantic].publish(
                telemetry_sample_to_message(
                    sample,
                    capability,
                    ros_now_ns=ros_now_ns,
                )
            )
            self._mark_published(semantic.value, _metadata_signature(sample.meta))
            publications += 1

        force = snapshot.force
        force_capability = capabilities.telemetry_channel(TelemetrySemantic.FORCE)
        if (
            force is not None
            and force_capability is not None
            and self._is_changed("force", force.meta)
        ):
            messages = force_sample_to_messages(
                force,
                force_capability,
                ros_now_ns=ros_now_ns,
            )
            if messages.telemetry is not None:
                self._force_publisher.publish(messages.telemetry)
                publications += 1
            if messages.tactile is not None:
                self._tactile_publisher.publish(messages.tactile)
                publications += 1
            if messages.telemetry is not None or messages.tactile is not None:
                self._mark_published("force", _metadata_signature(force.meta))

        stall_key = tuple(
            None
            if (sample := getattr(snapshot, attribute)) is None
            else _metadata_signature(sample.meta)
            for attribute in _STALL_ATTRIBUTES
        )
        if capabilities.stall_protection is not None and self._is_key_changed(
            "stall_protection", stall_key
        ):
            message = stall_protection_to_message(
                snapshot,
                capabilities,
                ros_now_ns=ros_now_ns,
            )
            if message is not None:
                self._stall_publisher.publish(message)
                self._mark_published("stall_protection", stall_key)
                publications += 1

        info = snapshot.device_info
        if info is not None and self._is_changed("device_info", info.meta):
            self._device_info_publisher.publish(
                device_info_to_message(info, ros_now_ns=ros_now_ns)
            )
            self._mark_published("device_info", _metadata_signature(info.meta))
            publications += 1
        return publications

    def reset(self) -> None:
        """Force the current cache to be emitted after the next activation."""

        self._published.clear()

    def destroy(self) -> None:
        """Destroy every publisher owned by this interface."""

        for publisher in (
            self._angles_publisher,
            *self._telemetry_publishers.values(),
            self._force_publisher,
            self._tactile_publisher,
            self._stall_publisher,
        ):
            self._node.destroy_lifecycle_publisher(publisher)
        self._node.destroy_publisher(self._capabilities_publisher)
        self._node.destroy_publisher(self._device_info_publisher)
        self._telemetry_publishers.clear()
        self._published.clear()

    def _is_changed(self, key: str, metadata: SampleMeta) -> bool:
        return self._is_key_changed(key, _metadata_signature(metadata))

    def _is_key_changed(self, key: str, value: object) -> bool:
        return self._published.get(key) != value

    def _mark_published(self, key: str, value: object) -> None:
        self._published[key] = value


def _metadata_signature(metadata: SampleMeta) -> tuple[object, ...]:
    return (
        metadata.sequence,
        metadata.valid,
        metadata.stale,
        metadata.connection_generation,
        metadata.origin,
        metadata.within_declared_range,
    )


__all__ = ["SnapshotPublisher"]
