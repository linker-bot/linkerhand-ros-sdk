"""QoS contracts shared by the Linkerbot hand driver and clients."""

from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

HIGH_RATE_PUBLISHER_EVENTS = PublisherEventCallbacks(use_default_callbacks=False)
HIGH_RATE_SUBSCRIPTION_EVENTS = SubscriptionEventCallbacks(use_default_callbacks=False)

COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=32,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

METADATA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

__all__ = [
    "COMMAND_QOS",
    "HIGH_RATE_PUBLISHER_EVENTS",
    "HIGH_RATE_SUBSCRIPTION_EVENTS",
    "METADATA_QOS",
    "STATE_QOS",
    "STATUS_QOS",
]
