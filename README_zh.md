# Linkerbot Hand ROS 2

本仓库包含两个可由 ROS 2 独立发现的 package：

- [`linkerbot_hand_msgs`](./linkerbot_hand_msgs)：类型化消息和服务。
- [`linkerbot_hand_ros`](./linkerbot_hand_ros)：Lifecycle 驱动和配套工具。

## 快速启动

完整依赖链为
`linkerbot-transport -> linkerbot-hand -> linkerbot_hand_msgs/linkerbot_hand_ros`。
Transport 和 Hand 使用已发布 Python 包；两个 ROS package 从同一个签名 `v1.0.0` tag
通过 `colcon` 构建。以下命令适用于已经配置 ROS 2 apt 软件源的 Ubuntu 24.04：

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
python3 -m venv --system-site-packages ~/venvs/linkerbot-hand-ros-1.0.0
source ~/venvs/linkerbot-hand-ros-1.0.0/bin/activate
python -m pip install --upgrade pip
python -m pip install \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-transport[all]==1.0.0' \
  'linkerbot-hand==1.0.0'

mkdir -p ~/linkerbot_ws/src
cd ~/linkerbot_ws/src
git clone --branch v1.0.0 --depth 1 \
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

验证应显示两个 Python 包均为 `1.0.0`、两个 ROS package 的工作空间安装前缀，以及
`hand_node` 等可执行程序。该步骤不连接设备，也不会发送 CAN 帧。每个新终端依次执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/linkerbot-hand-ros-1.0.0/bin/activate
source ~/linkerbot_ws/install/setup.bash
```

ROS 2 apt 软件源的首次配置见
[ROS 2 Jazzy 官方安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)。
实机 CAN 配置、启动参数和接口检查见下方驱动文档。

安装、运行和接口说明见[驱动中文文档](./linkerbot_hand_ros/README_zh.md)，英文说明见
[driver README](./linkerbot_hand_ros/README.md)。

版本变更分别记录在 [`linkerbot_hand_msgs/CHANGELOG.rst`](./linkerbot_hand_msgs/CHANGELOG.rst)
和 [`linkerbot_hand_ros/CHANGELOG.rst`](./linkerbot_hand_ros/CHANGELOG.rst)。
