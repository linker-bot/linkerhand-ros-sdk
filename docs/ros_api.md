# ROS 2 API

The lifecycle driver uses the SDK capability model as its only source for joint
order, ranges, sensor support, and percentage mapping. All models expose
`100=fully open` and `0=fully closed`. Values are percentages, not radians.
`raw_angles` always means exact unified SDK protocol values and never classic
`standard_raw` or the classic user LUT coordinate.

Configure performs no device I/O. Activate opens the one `HandDevice` and
starts SDK polling. ROS state timers only call `get_snapshot()` and
`get_health()`.

## Parameters

| Name | Default | Description |
| --- | --- | --- |
| `model` | `l6` | `o6/l6/l20lite/l25/o20/o30i/l30` |
| `side` | `left` | `left/right/unknown` |
| `transport_options_json` | `{}` | SDK transport constructor options; preferred |
| `command_rate_hz` | `100.0` | Maximum SDK continuous-command dispatch rate |
| `auto_reconnect` | `false` | Enable SDK exponential-backoff recovery |
| `publish_rate_hz` | `60.0` | Cached ROS publication rate; zero pauses state topics |
| `angles_hz` | `60.0` | SDK angle polling rate; zero disables |
| `speeds_hz` | `0.0` | SDK speed polling rate; zero disables |
| `torques_hz` | `0.0` | SDK torque polling rate; zero disables |
| `force_hz` | `0.0` | SDK force polling rate; no ROS force message yet |
| `temperature_hz` | `1.0` | SDK temperature polling rate |
| `current_hz` | `10.0` | SDK current polling rate |
| `faults_hz` | `5.0` | SDK fault polling rate |
| `diagnostic_rate` | `1.0` | `/diagnostics` publication rate; zero pauses |

Requested sensor rates are capped at each SDK capability's recommended rate;
unsupported sensors are disabled. Legacy `interface_name`, `interface_type`,
and `library_path` parameters apply only when `transport_options_json` is `{}`.
The launch-only arguments are `namespace` and `autostart`.

Polling rates, `publish_rate_hz`, and `diagnostic_rate` are runtime-adjustable.
An active sensor-rate update atomically replaces the SDK polling plan without
reopening the transport. Zero stops that sensor's queries and state topic.
Other parameters remain configure-time snapshots.

## Topics

| Topic | Type | Direction |
| --- | --- | --- |
| `~/command/angles` | `std_msgs/msg/Float64MultiArray` | Subscribe while active |
| `~/command/raw_angles` | `std_msgs/msg/Float64MultiArray` | Subscribe exact protocol values while active |
| `~/command/speeds` | `std_msgs/msg/Float64MultiArray` | Subscribe while active |
| `~/command/torques` | `std_msgs/msg/Float64MultiArray` | Subscribe while active |
| `~/command/accepted/angles` | `std_msgs/msg/Float64MultiArray` | Publish SDK acceptance |
| `~/command/accepted/raw_angles` | `std_msgs/msg/Float64MultiArray` | Publish protocol-raw SDK acceptance |
| `~/command/accepted/speeds` | `std_msgs/msg/Float64MultiArray` | Publish SDK acceptance |
| `~/command/accepted/torques` | `std_msgs/msg/Float64MultiArray` | Publish SDK acceptance |
| `~/state/angles` | `std_msgs/msg/Float64MultiArray` | Publish fresh cached percentages |
| `~/state/raw_angles` | `std_msgs/msg/Float64MultiArray` | Publish exact protocol values from the same cached sample |
| `~/state/speeds` | `std_msgs/msg/Float64MultiArray` | Publish fresh speed cache when supported |
| `~/state/torques` | `std_msgs/msg/Float64MultiArray` | Publish fresh torque cache when supported |
| `~/state/temperature` | `std_msgs/msg/Float64MultiArray` | Publish fresh cache when supported |
| `~/state/current` | `std_msgs/msg/Float64MultiArray` | Publish fresh cache when supported |
| `~/state/fault` | `std_msgs/msg/Float64MultiArray` | Publish fresh cache when supported |
| `~/joint_names` | `std_msgs/msg/String` | Publish SDK joint order |
| `~/model` | `std_msgs/msg/String` | Publish normalized model |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | SDK health and command stages |

An accepted command is only at SDK `ACCEPTED` stage. It has not necessarily
been dispatched, acknowledged by the device, or physically completed. Recent
`accepted/dispatched/acknowledged/failed/superseded` stages are exposed in
diagnostics, including the most recent `clear_faults` service command.
Continuous commands use latest-wins at both ROS QoS and SDK layers.

An unavailable percentage readback is represented by `NaN` to preserve array
shape. Invalid or stale samples are not published.

## Services

| Service | Type | Description |
| --- | --- | --- |
| `~/read_now` | `std_srvs/srv/Trigger` | Republish fresh angle cache without device I/O |
| `~/clear_faults` | `std_srvs/srv/Trigger` | Asynchronously submit clear-faults when supported |

## Command Ranges

| Model | Angles | Speeds | Torques |
| --- | --- | --- | --- |
| O6, L6, L20Lite, L25 | 0–100 | 0–100 | 0–100 |
| O20 | 0–100 | 0–100 integer | 0–1000 integer |
| O30I | 0–100 | 0–255 integer | 0–255 integer |
| L30 | 0–100 | 1–250 integer | 60–800 integer |

For full lifecycle, diagnostics, joint orders, and business semantics, see the
[Chinese API reference](zh/ros_api.md).
