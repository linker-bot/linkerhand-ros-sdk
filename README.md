# Linkerbot Hand ROS 2

[English](README.md) | [简体中文](README_zh.md)

Production-oriented ROS 2 Jazzy driver for Linkerbot O6, L6, L20Lite, L25,
O20, O30I, and L30 hands. Model capabilities, polling, command scheduling, and
recovery come from the split `linkerbot-hand` Python SDK.

The driver is a `LifecycleNode`. Configure creates a no-I/O `HandDevice`;
activate opens the single device connection and starts SDK-owned polling. ROS
timers only publish cached snapshots, and command callbacks submit latest-wins
targets without waiting for transport or device acknowledgements.

Flexion axes use `100=fully open` and `0=fully closed`. Directional axes such
as abduction and rotation use the capability-declared zero/hundred endpoints;
they must not be described as open/closed.
Separate `raw_angles` command and cached-state topics expose exact protocol
coordinates without applying the classic user LUT.

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy
- Python 3.10 or newer
- `linkerbot-hand>=0.1.0,<0.2.0`
- A configured SocketCAN/CAN FD interface or supported vendor library

The released `linkerbot-hand v0.1.0` supports the unified API. Its successor
contains corrected physical units for speed and current limits; use the latest
SDK source for hardware unit validation until that version is released.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install 'linkerbot-hand>=0.1.0,<0.2.0'
```

For local SDK development, install the independent repositories. Installing
`linkerbot-hand` resolves its compatible `linkerbot-transport` dependency:

```bash
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-transport.git
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand.git
python3 -m pip install -e ./linkerbot-transport -e ./linkerbot-hand
```

## Build

```bash
mkdir -p ~/linkerbot_ws/src
cd ~/linkerbot_ws/src
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand-ros.git
cd ~/linkerbot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/linkerbot_ws/install/setup.bash
```

## Run

For an L6 on `can0`:

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left \
  transport_options_json:='{"interface_name":"can0","interface_type":"socketcan"}'
ros2 lifecycle get /linkerbot_hand
```

Polling and cached-state publication can be adjusted after launch without
reopening the device connection:

```bash
ros2 param set /linkerbot_hand torques_hz 30.0
ros2 param set /linkerbot_hand torques_hz 0.0
ros2 param set /linkerbot_hand publish_rate_hz 0.0
ros2 param set /linkerbot_hand publish_rate_hz 60.0
```

Static connection parameters are accepted only while unconfigured. Run
`deactivate` and `cleanup` before changing model, side, transport, command
rate, or reconnect policy.

The launch file defaults to `autostart:=true`. Set it to `false` when an
external lifecycle manager owns transitions.

Use the persistent controller for repeated commands:

```bash
ros2 run linkerbot_hand_ros angle_controller
```

The installed `frequency_benchmark` executable measures command delivery and
real SDK sample-sequence rates separately. See the
[Chinese frequency benchmark guide](docs/zh/frequency_benchmark.md) before
running 50–100 Hz tests on hardware.

See [the Chinese integration guide](docs/zh/business_integration.md) and the
[ROS API reference](docs/zh/ros_api.md) for lifecycle, parameters, topics,
services, diagnostics, and command semantics.

## Safety

Percentage values are not radians. A DDS acknowledgement, SDK acceptance,
protocol acknowledgement, and physical completion are distinct events. Verify
model, side, joint order, bus configuration, emergency stop, conservative
speed, and torque limits before commanding real hardware.

## Development

Hardware-independent tests use the SDK `FakeBackendFactory`:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest
python3 -m compileall linkerbot_hand_ros launch
```
