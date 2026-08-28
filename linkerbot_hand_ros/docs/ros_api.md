# ROS 2 API Reference

The driver exposes the Hand SDK 1.x contract through typed interfaces. Topic
arrays always follow channel names published in `~/capabilities`; there is no
ROS model table.

## Lifecycle

| Transition | Device behavior |
| --- | --- |
| configure | Construct one `HandDevice`; zero transport open and zero device I/O |
| activate | Open, verify identity/firmware, start SDK polling, enable commands |
| deactivate | Remove command subscriptions and stop SDK polling; keep connection |
| cleanup/shutdown | Bounded `close()` and ROS entity destruction |

## Parameters

Static parameters are accepted only while unconfigured:

`model`, `side`, `transport_json`, `command_rate_hz`,
`discrete_queue_capacity`, `mailbox_capacity`, `route_queue_capacity`,
`identity_timeout_s`, `transaction_timeout_s`, `shutdown_timeout_s`,
`auto_reconnect`, `reconnect_initial_delay_s`, and
`reconnect_max_delay_s`.

Runtime rates are `publish_rate_hz`, `diagnostic_rate_hz`, `angles_hz`,
`speeds_hz`, `force_hz`, `temperature_hz`, `current_hz`, `faults_hz`,
`torques_hz`, and `voltage_hz`. Zero disables that SDK polling channel;
`publish_rate_hz=0` only pauses ROS state publication. Runtime capability
fallback is still checked from SDK memory and republished at 10 Hz without
performing device I/O.

The default transport is:

```json
{"backend":"socketcan","channel":"can0"}
```

## Multi-Device Process

`multi_hand.launch.py` loads multiple `/namespace/linkerbot_hand` Lifecycle
nodes into one Python process from a ROS parameter file. The process-level
parameters are `device_namespaces`, `autostart`, and `executor_threads`. With
autostart enabled, all nodes configure before any node activates; a failure
rolls the group back in reverse order. Identical physical transport keys share
one process-local runtime. Separate driver processes cannot share that arbiter.

## Command Topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `~/command/angles` | `linkerbot_hand_msgs/msg/HandJointCommand` | Percent or protocol-raw angle target |
| `~/command/speeds` | `linkerbot_hand_msgs/msg/HandRawCommand` | Protocol-raw configured speed |
| `~/command/accelerations` | `linkerbot_hand_msgs/msg/HandRawCommand` | Protocol-raw configured acceleration |
| `~/command/current_limits` | `linkerbot_hand_msgs/msg/HandRawCommand` | Protocol-raw current limit |
| `~/command/effort_limits` | `linkerbot_hand_msgs/msg/HandRawCommand` | Protocol-raw dimensionless effort limit |
| `~/command/torques` | `linkerbot_hand_msgs/msg/HandRawCommand` | True torque only when capability supports it |
| `~/command/motion_times` | `linkerbot_hand_msgs/msg/HandRawCommand` | Protocol-raw motion-time setting |
| `~/command/status` | `linkerbot_hand_msgs/msg/HandCommandStatus` | SDK command stage history |

Continuous command topics use best-effort, depth-one QoS. Unsupported command
topics are not subscribed while active. `ACCEPTED` means admitted to the SDK;
`DISPATCHED` means transport send succeeded; `ACKNOWLEDGED` requires an explicit
device-protocol ACK. None of these means mechanical arrival.

Each command carries a non-zero `client_id`, a client-local `user_sequence`,
and `publisher_stamp`. `HandCommandStatus` echoes that composite key and adds
`driver_received_stamp` plus the complete SDK transition history. Clients must
match both ID fields; a final `ACKNOWLEDGED` status does not replace the explicit
`DISPATCHED` transition when measuring actual sends.

## State Topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `~/capabilities` | `HandCapabilities` | Declared, then verified capability metadata |
| `~/device_info` | `HandDeviceInfo` | Cached verified identity and version fields |
| `~/state/angles` | `HandJointState` | Raw, percent, and optional radians from one sample |
| `~/state/<semantic>` | `HandTelemetry` | Protocol-raw typed telemetry/configuration sample |
| `~/state/force` | `HandTelemetry` | Scalar force when the profile provides it |
| `~/state/tactile` | `HandTactileState` | One complete multi-finger tactile transaction |
| `~/state/stall_protection` | `HandStallProtectionState` | Three independent raw stall settings |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Zero-I/O SDK health and transport metrics |

`<semantic>` can be `measured_speeds`, `configured_speeds`,
`configured_accelerations`, `currents`, `current_limits`, `effort_limits`,
`torques`, `temperatures`, `faults`, `voltages`, or `motion_times`, depending on
verified capability. A state message is published only when its SDK sequence or
validity signature changes.

When an optional query is rejected by the connected firmware, the SDK retains
the declared protocol metadata and adds its semantic to
`HandCapabilities.firmware_unsupported_telemetry`. The driver republishes that
runtime change with transient-local durability; consumers should treat the
latest capability message as authoritative for the current connection
generation.

Every state carries collection time, sequence, age, validity, staleness,
connection generation, origin, and range status. Protocol raw values are not
converted to RPM, mA, N.m, or other physical units by ROS.
`ORIGIN_DEVICE_COMMAND_FEEDBACK` identifies verified current state returned by
a command; it is neither a target echo nor a command acknowledgement.

## Services

| Service | Type | Result semantics |
| --- | --- | --- |
| `~/get_device_info` | `GetDeviceInfo` | Admin worker performs one fresh identity transaction |
| `~/get_stall_protection` | `GetStallProtection` | Freshly reads all three supported stall settings |
| `~/set_stall_detection_times` | `SetRawValues` | Asynchronous raw configuration acceptance |
| `~/set_stall_thresholds` | `SetRawValues` | Asynchronous raw configuration acceptance |
| `~/set_stall_hold_currents` | `SetRawValues` | Asynchronous raw configuration acceptance |
| `~/save_parameters` | `SubmitCommand` | Capability-gated non-volatile save; observe command status for protocol ACK |
| `~/clear_faults` | `SubmitCommand` | Asynchronous discrete command acceptance |
| `~/enable`, `~/disable`, `~/calibrate` | `SubmitCommand` | Capability-gated asynchronous acceptance |
| `~/recover` | `Recover` | Bounded identity-verified safe recovery |

Active reads run in one separate bounded admin worker. Control callbacks and
state publishers never wait for these transactions. Services return stable
`error_code` fields; clients must not parse error text.

Diagnostics expose each channel's requested and achieved rate, completed read
transactions, command-feedback samples, policy-counted feedback samples,
feedback policy, deadlines, timeouts, and cached data age. These counters let
benchmarks distinguish active angle queries from verified command feedback.

See the [Chinese API reference](zh/ros_api.md) for examples and QoS details.
