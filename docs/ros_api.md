# ROS 2 API

The lifecycle driver uses the SDK capability model as its only source for joint
order, ranges, sensor support, and percentage mapping. Flexion axes expose
`100=open` and `0=closed`; directional axes use capability-declared position
endpoints. Values are normalized percentages, not radians.
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
| `voltage_hz` | `0.0` | SDK voltage polling rate; zero disables |
| `acceleration_hz` | `0.0` | SDK acceleration polling rate; zero disables |
| `motion_time_hz` | `0.0` | SDK motion-time polling rate; zero disables |
| `diagnostic_rate` | `1.0` | `/diagnostics` publication rate; zero pauses |

Requested rates are passed to the SDK unchanged for supported sensors;
`recommended_hz` is guidance rather than a limit. Unsupported sensors are
disabled. Legacy `interface_name`, `interface_type`, and `library_path`
parameters apply only when `transport_options_json` is `{}`.
The launch-only arguments are `namespace` and `autostart`.

Polling rates, `publish_rate_hz`, and `diagnostic_rate` are runtime-adjustable.
An active sensor-rate update atomically replaces the SDK polling plan without
reopening the transport. Zero stops that sensor's queries and state topic.
Static parameters are rejected after configure. Deactivate and cleanup before
changing model, side, transport, command rate, or reconnect policy.

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
| `~/state/voltage` | `std_msgs/msg/Float64MultiArray` | Publish fresh voltage cache when supported |
| `~/state/acceleration` | `std_msgs/msg/Float64MultiArray` | Publish fresh acceleration cache when supported |
| `~/state/motion_time` | `std_msgs/msg/Float64MultiArray` | Publish fresh motion-time cache when supported |
| `~/joint_names` | `std_msgs/msg/String` | Publish SDK joint order |
| `~/model` | `std_msgs/msg/String` | Publish normalized model |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | SDK health and command stages |

An accepted command is only at SDK `ACCEPTED` stage. It has not necessarily
been dispatched, acknowledged by the device, or physically completed. Recent
`accepted/dispatched/acknowledged/failed/superseded` stages are exposed in
diagnostics, including management service commands.
Continuous commands use latest-wins at both ROS QoS and SDK layers.

An unavailable percentage readback is represented by `NaN` to preserve array
shape. Invalid or stale samples are not published.

## Services

| Service | Type | Description |
| --- | --- | --- |
| `~/read_now` | `std_srvs/srv/Trigger` | Republish fresh angle cache without device I/O |
| `~/clear_faults` | `std_srvs/srv/Trigger` | Asynchronously submit clear-faults when supported |
| `~/enable` | `std_srvs/srv/Trigger` | Asynchronously submit device enable when supported |
| `~/disable` | `std_srvs/srv/Trigger` | Asynchronously submit device disable when supported |
| `~/calibrate` | `std_srvs/srv/Trigger` | Asynchronously submit calibration when supported |

Trigger success means SDK acceptance only. Inspect diagnostics for dispatch or
protocol acknowledgement. Tactile force remains in the SDK cache but is not
flattened into a numeric array; it requires a dedicated structured ROS message.

## Command Ranges

| Model | Angles | Speeds | Torques |
| --- | --- | --- | --- |
| L6 | 0–100 | 0–100 dimensionless | 0–1400 mA |
| O6 | 0–100 | 0–180 RPM | 0–1000 mA |
| L20Lite, L25 | 0–100 | 0–100 dimensionless | 0–100 dimensionless |
| O20 | 0–100 | target 0–100 integer | 0–1000 integer at 6.5 mA/LSB |
| O30I | 0–100 | 0–255 integer | 0–255 integer |
| L30 | 0–100 | 1–250 protocol units | 390–5200 mA, quantized to 6.5 mA |

L30 current state is mA; each sample's protocol integer is retained in the SDK
raw cache. L6 speed is write-only: its accepted target is available on the
accepted command topic, while no measured speed state is published. O6 speed
state is RPM and torque state is mA; O20 speed state is RPM. Exact protocol
values remain in the SDK sensor sample raw cache and are not duplicated as ROS
numeric state topics.

For full lifecycle, diagnostics, joint orders, and business semantics, see the
[Chinese API reference](zh/ros_api.md).
