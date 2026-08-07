# Linkerbot Hand ROS 2

[English](README.md) | [简体中文](README_zh.md)

ROS 2 Python driver for Linkerbot dexterous hands. The package wraps the
[`linkerbot`](https://pypi.org/project/linkerbot/) Python SDK and supports
O6, L6, L20Lite, L25, O20, O30I, and L30 hands through one node.

## Requirements

- ROS 2 Humble or newer
- Python 3.10 or newer
- A configured SocketCAN/CAN FD interface, or the vendor CAN FD library
- `linkerbot` Python SDK 0.5.4 or newer

Install the SDK into the Python environment used by ROS 2:

```bash
python3 -m pip install 'linkerbot>=0.5.4'
```

## Build

```bash
mkdir -p ~/linkerbot_ws/src
cd ~/linkerbot_ws/src
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand-ros.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

For a classic CAN hand such as L6:

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left interface_name:=can0
```

For a SocketCAN CAN FD hand such as L30:

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l30 interface_type:=socketcan interface_name:=can0
```

Parameters can also be loaded from
[`config/hand.yaml`](config/hand.yaml). See
[`docs/ros_api.md`](docs/ros_api.md) for the complete interface and model joint
orders.

Runnable Python examples are installed with the package:

```bash
ros2 run linkerbot_hand_ros angle_command --values 0 0 0 0 0 0
ros2 run linkerbot_hand_ros state_monitor
```

## Safety

Angle commands are percentages in the closed interval `[0, 100]`, following
the Python SDK: `0` is open and `100` is closed. Speed and torque ranges are
model-specific and documented in the ROS API. Verify joint order, hand model,
CAN interface, and conservative speed/torque values before commanding hardware.

## Development

Run hardware-independent checks with:

```bash
python3 -m pytest
python3 -m compileall linkerbot_hand_ros launch
```

Hardware communication is intentionally excluded from automated tests.
