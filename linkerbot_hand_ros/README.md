# Linkerbot Hand ROS 2

[English](README.md) | [简体中文](README_zh.md)

ROS 2 Jazzy Lifecycle driver for Linkerbot O6, L6, L20Lite, L25, O20,
O30I, and L30 hands. The driver targets the single `HandDevice` 1.x API and
uses the SDK capability object as its only model specification.

The ownership boundary is strict:

- configure constructs descriptors and ROS entities without device I/O;
- activate calls `open()` and starts SDK-owned polling;
- command callbacks call non-blocking `set_*()` methods only;
- state publication reads `get_snapshot()` and emits each SDK sequence once;
- diagnostics reads `get_health()` only;
- deactivate stops polling while cleanup and shutdown close the device.

There are no compatibility array topics or ROS-owned device query timers.
Protocol raw, percent, and physical-angle values remain separate in typed
messages. Flexion percent uses `0=flexed/closed` and `100=extended/open`;
directional axes use the endpoint labels published in `HandCapabilities`.

## Requirements

- Ubuntu 24.04 and ROS 2 Jazzy
- Python 3.10 or newer
- `linkerbot-hand>=1.0.0.dev0,<2.0.0`
- a configured SocketCAN/CAN FD interface or supported vendor adapter

Install a published SDK build with both the private package index and public
PyPI available for third-party dependencies:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-hand>=1.0.0.dev0,<2.0.0'
```

For unreleased 1.x development, install the independent source repositories
into the same Python environment used by ROS:

```bash
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-transport.git
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand.git
python3 -m pip install -e ./linkerbot-transport -e ./linkerbot-hand
python3 -c 'import linkerbot_hand; print(linkerbot_hand.__file__)'
```

## Build

```bash
mkdir -p ~/linkerbot_ws/src
cd ~/linkerbot_ws/src
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand-ros.git
cd ~/linkerbot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --symlink-install
source ~/linkerbot_ws/install/setup.bash
```

Any unresolved rosdep key is an error. A `pkg_resources is deprecated` line
from Ubuntu's rosdep launcher is only a warning.

## Run

Start a left L6 on `can0`:

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left \
  transport_json:='{"backend":"socketcan","channel":"can0"}'
```

L30 has no left/right variant, so pass an empty side:

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l30 side:='' \
  transport_json:='{"backend":"socketcan","channel":"can0"}'
```

Inspect the verified metadata and state:

```bash
ros2 lifecycle get /linkerbot_hand
ros2 topic echo --once /linkerbot_hand/capabilities
ros2 topic echo --once /linkerbot_hand/device_info
ros2 topic echo --once /linkerbot_hand/state/angles
ros2 topic echo /linkerbot_hand/command/status
```

Send one percent target with the typed message:

```bash
ros2 topic pub --once /linkerbot_hand/command/angles \
  linkerbot_hand_msgs/msg/HandJointCommand \
  '{user_sequence: 1, representation: 1, values: [50, 50, 50, 50, 50, 50]}'
```

`representation=1` means percent and `representation=2` means exact protocol
raw. The values always follow the capability joint order. Use the persistent
Python client for control loops:

```bash
ros2 run linkerbot_hand_ros angle_controller
```

The frequency tool evaluates angle command publication, SDK terminal stages,
device sample sequences, and optional complete tactile transactions separately:

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --angle-rates 50 100 150 200 --duration 180 --repetitions 3
```

See [ROS API](docs/ros_api.md), the
[Chinese integration guide](docs/zh/business_integration.md), and the
[frequency benchmark guide](docs/zh/frequency_benchmark.md).

## Safety

`ACCEPTED`, `DISPATCHED`, protocol `ACKNOWLEDGED`, and mechanical arrival are
different states. A successful service response for a command means accepted,
not delivered. Determine arrival from valid, non-stale feedback in the current
connection generation. Recovery never replays pre-disconnect targets.

## Development

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```
