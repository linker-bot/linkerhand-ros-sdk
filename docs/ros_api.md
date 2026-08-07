# ROS 2 API

Angle command and state values use the SDK's logical percentage range: `0.0`
is open and `100.0` is closed. Speed and torque ranges follow each device
protocol. Array order is published once per second on `~/joint_names` as a
comma-separated string.

## Parameters

| Name | Default | Description |
| --- | --- | --- |
| `model` | `l6` | `o6`, `l6`, `l20lite`, `l25`, `o20`, `o30i`, or `l30` |
| `side` | `left` | `left` or `right` where supported |
| `interface_name` | `can0` | SocketCAN interface |
| `interface_type` | `socketcan` | SDK transport (`socketcan` or `ctypes`) |
| `library_path` | empty | Optional vendor CAN FD shared library path |
| `poll_rate` | `30.0` | Angle polling rate in Hz |
| `timeout_ms` | `100.0` | SDK blocking read timeout |
| `publish_optional_state` | `true` | Read temperature/current/fault when supported |
| `optional_state_rate` | `1.0` | Optional state polling rate in Hz |

## Command Ranges

| Model | Angles | Speeds | Torques |
| --- | --- | --- | --- |
| O6, L6, L20Lite, L25 | 0–100 | 0–100 | 0–100 |
| O20 | 0–100 | 0–100 integer | 0–1000 integer |
| O30I | 0–100 | 0–255 integer | 0–255 integer |
| L30 | 0–100 | 1–250 integer | 60–800 integer |

## Topics

| Topic | Type | Direction |
| --- | --- | --- |
| `~/command/angles` | `std_msgs/msg/Float64MultiArray` | Subscribe |
| `~/command/speeds` | `std_msgs/msg/Float64MultiArray` | Subscribe |
| `~/command/torques` | `std_msgs/msg/Float64MultiArray` | Subscribe |
| `~/state/angles` | `std_msgs/msg/Float64MultiArray` | Publish |
| `~/state/temperature` | `std_msgs/msg/Float64MultiArray` | Publish when available |
| `~/state/current` | `std_msgs/msg/Float64MultiArray` | Publish when available |
| `~/state/fault` | `std_msgs/msg/Float64MultiArray` | Publish when available |
| `~/joint_names` | `std_msgs/msg/String` | Publish |

## Services

| Service | Type | Description |
| --- | --- | --- |
| `~/read_now` | `std_srvs/srv/Trigger` | Read and publish angles immediately |
| `~/clear_faults` | `std_srvs/srv/Trigger` | Clear faults when supported by the model |

## Joint Counts

O6 and L6 have 6 joints, L20Lite has 10, L25 and O20 have 16, L30 has 17,
and O30I has 20. Exact names and order are defined by `MODEL_SPECS` in the
adapter and published by the running node.
