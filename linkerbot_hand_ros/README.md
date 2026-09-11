# Linkerbot Hand ROS 2

[English](README.md) | [简体中文](README_zh.md)

ROS 2 Jazzy Lifecycle driver for Linkerbot O6, L6, L20Lite, L20, O20,
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
Runtime optional-query fallback is republished through
`firmware_unsupported_telemetry`; supported non-volatile configuration is
submitted through the capability-gated `save_parameters` service.

## Requirements

- Ubuntu 24.04 and ROS 2 Jazzy
- Python 3.10 or newer
- `linkerbot-transport>=1.0.0,<2.0.0`
- `linkerbot-hand>=2.0.0,<3.0.0`
- a configured SocketCAN/CAN FD interface or supported vendor adapter

First-time users can run the repository
[quick start](../README.md#quick-start). For a manual installation, create an
isolated environment that can read the system ROS packages and explicitly
install both Transport and Hand:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages ~/venvs/linkerbot-hand-ros-2.0.0
source ~/venvs/linkerbot-hand-ros-2.0.0/bin/activate
python -m pip install \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-transport[all]==1.0.0' \
  'linkerbot-hand==2.0.0'
```

For unreleased 1.x development, install the independent source repositories
into the same Python environment used by ROS:

```bash
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-transport.git
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand.git
python -m pip install -e ./linkerbot-transport -e ./linkerbot-hand
python -c 'import linkerbot_hand; print(linkerbot_hand.__file__)'
```

## Build

`linkerbot_hand_msgs` contains ROSIDL interfaces and must be generated for the
installed ROS distribution; it is therefore delivered in the tagged ROS source
release rather than inside the platform-independent driver wheel. Build both
ROS packages from the same `v2.0.0` tag:

```bash
mkdir -p ~/linkerbot_ws/src
cd ~/linkerbot_ws/src
git clone --branch v2.0.0 --depth 1 \
  https://gitea.linkerhub.work/zhuzhixiang/linkerbot-hand-ros.git
cd ~/linkerbot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
python -m colcon build --symlink-install
source ~/linkerbot_ws/install/setup.bash

python -c 'import linkerbot_transport, linkerbot_hand; print("Python dependencies: OK")'
ros2 pkg prefix linkerbot_hand_msgs
ros2 pkg prefix linkerbot_hand_ros
ros2 interface show linkerbot_hand_msgs/msg/HandJointCommand >/dev/null
ros2 pkg executables linkerbot_hand_ros
```

Any unresolved rosdep key is an error. A `pkg_resources is deprecated` line
from Ubuntu's rosdep launcher is only a warning.
This verification checks dependencies, ROSIDL interfaces, the ament index, and
program entry points without opening a device or sending a CAN frame.

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

Run multiple hands in one process with a ROS parameter file:

```bash
ros2 launch linkerbot_hand_ros multi_hand.launch.py
```

The installed `config/multi_hand.yaml` example creates
`/l6_left/linkerbot_hand` and `/o6_right/linkerbot_hand`. Pass
`config:=/absolute/path/to/hands.yaml` for a deployment-specific file. Hands
with the same `transport_json` physical key share one transport runtime, RX
reactor, and TX arbiter. Starting `hand.launch.py` twice creates two processes
and therefore cannot provide process-local bus arbitration.

Inspect the verified metadata and state:

```bash
ros2 lifecycle get /linkerbot_hand
ros2 topic echo --once /linkerbot_hand/capabilities
ros2 topic echo --once /linkerbot_hand/device_info
ros2 topic echo --once /linkerbot_hand/state/angles
ros2 topic echo /linkerbot_hand/command/status
```

`/command/status` uses `HandCommandStatusBatch`; consumers must iterate every
changed status in `statuses[]` and correlate both `client_id` and
`user_sequence`.

Send one percent target with the typed message:

```bash
ros2 topic pub --once /linkerbot_hand/command/angles \
  linkerbot_hand_msgs/msg/HandJointCommand \
  '{client_id: 1, user_sequence: 1, representation: 1, values: [50, 50, 50, 50, 50, 50]}'
```

`representation=1` means percent and `representation=2` means exact protocol
raw. The values always follow the capability joint order. Use the persistent
Python client for control loops:

```bash
ros2 run linkerbot_hand_ros angle_controller
```

The frequency tool evaluates publish, driver receive, SDK acceptance, actual
dispatch, terminal stages, device sample sequences, and optional complete
tactile transactions separately:

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --angle-feedback query_only --angle-rates 50 100 150 200 \
  --duration 180 --repetitions 3
```

The production driver uses a single-threaded executor to avoid Python GIL
contention between short callbacks. SDK cache revisions wake a dedicated state
publisher immediately, so equal-rate timer aliasing does not discard feedback.
`publish_rate_hz` controls idle cache checks for age/stale transitions; zero
pauses ROS state publication. An unchanged sequence is never republished.
In `query_only`, command feedback remains in SDK cache and metrics but
`~/state/angles` publishes active-query responses only.

See [ROS API](docs/ros_api.md), the
[Chinese integration guide](docs/zh/business_integration.md), and the
[frequency benchmark guide](docs/zh/frequency_benchmark.md). Release history is
maintained in [CHANGELOG.rst](CHANGELOG.rst); every user-facing pull request
must update its `Unreleased` section. Message or service contract changes must
also update `linkerbot_hand_msgs/CHANGELOG.rst`.

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
