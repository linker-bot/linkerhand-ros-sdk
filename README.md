# Linkerbot Hand ROS 2

This repository contains two independently discoverable ROS 2 packages:

- [`linkerbot_hand_msgs`](./linkerbot_hand_msgs): typed messages and services.
- [`linkerbot_hand_ros`](./linkerbot_hand_ros): the Lifecycle driver and tools.

## Quick Start

The complete dependency chain is
`linkerbot-transport -> linkerbot-hand -> linkerbot_hand_msgs/linkerbot_hand_ros`.
Transport and Hand use published Python packages, while both ROS packages are
built with `colcon` from the same signed `v2.0.0` tag. On Ubuntu 24.04 with the
ROS 2 apt repository configured, run:

```bash
sudo apt update
sudo apt install -y \
  git can-utils python3-venv python3-rosdep \
  python3-colcon-common-extensions ros-jazzy-ros-base

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages ~/venvs/linkerbot-hand-ros-2.0.0
source ~/venvs/linkerbot-hand-ros-2.0.0/bin/activate
python -m pip install --upgrade pip
python -m pip install \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-transport[all]==1.0.0' \
  'linkerbot-hand==2.0.0'

mkdir -p ~/linkerbot_ws/src
cd ~/linkerbot_ws/src
git clone --branch v2.0.0 --depth 1 \
  https://gitea.linkerhub.work/zhuzhixiang/linkerbot-hand-ros.git

cd ~/linkerbot_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
python -m colcon build --symlink-install
source ~/linkerbot_ws/install/setup.bash

python - <<'PY'
from importlib.metadata import version

import linkerbot_hand
import linkerbot_transport

print("linkerbot-transport", version("linkerbot-transport"))
print("linkerbot-hand", version("linkerbot-hand"))
print("python imports: OK")
PY
ros2 pkg prefix linkerbot_hand_msgs
ros2 pkg prefix linkerbot_hand_ros
ros2 interface show linkerbot_hand_msgs/msg/HandJointCommand >/dev/null
ros2 pkg executables linkerbot_hand_ros
```

Verification should report transport `1.0.0` and hand `2.0.0`, workspace prefixes
for both ROS packages, and executables including `hand_node`. It does not open
a device or send CAN frames. In each new terminal, source the environments in
this order:

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/linkerbot-hand-ros-2.0.0/bin/activate
source ~/linkerbot_ws/install/setup.bash
```

See the
[official ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
to configure the ROS apt repository for the first time. The driver guide below
covers real-device CAN setup, launch parameters, and runtime checks.

See the [driver README](./linkerbot_hand_ros/README.md) for installation and
usage. The [Chinese documentation](./linkerbot_hand_ros/README_zh.md) is also
available.

Release histories are maintained in
[`linkerbot_hand_msgs/CHANGELOG.rst`](./linkerbot_hand_msgs/CHANGELOG.rst) and
[`linkerbot_hand_ros/CHANGELOG.rst`](./linkerbot_hand_ros/CHANGELOG.rst).
