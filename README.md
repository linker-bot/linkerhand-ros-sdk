# Linkerbot Hand ROS 2

[English](README.md) | [简体中文](README_zh.md)

Production-oriented ROS 2 Jazzy driver for Linkerbot O6, L6, L20Lite, L25,
O20, O30I, and L30 hands. Model capabilities, polling, command scheduling, and
recovery come from the split `linkerbot-hand` Python SDK.

The driver is a `LifecycleNode`. Configure creates a no-I/O `HandDevice`;
activate opens the single device connection and starts SDK-owned polling. ROS
timers only publish cached snapshots, and command callbacks submit latest-wins
targets without waiting for transport or device acknowledgements.

All models expose the same percentage convention: `100=fully open` and
`0=fully closed`.

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy
- Python 3.10 or newer
- `linkerbot-hand>=0.1.0,<0.2.0`
- A configured SocketCAN/CAN FD interface or supported vendor library

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install 'linkerbot-hand>=0.1.0,<0.2.0'
```

For local SDK development:

```bash
python3 -m pip install \
  -e /home/lxqs/code/linkerbot-python-sdk/packages/linkerbot-common \
  -e /home/lxqs/code/linkerbot-python-sdk/packages/linkerbot-hand
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

The launch file defaults to `autostart:=true`. Set it to `false` when an
external lifecycle manager owns transitions.

Use the persistent controller for repeated commands:

```bash
ros2 run linkerbot_hand_ros angle_controller
```

See [the Chinese integration guide](docs/zh/business_integration.md) and the
[ROS API reference](docs/ros_api.md) for lifecycle, parameters, topics,
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
