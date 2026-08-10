# Linkerbot Hand ROS 2

[English](README.md) | [简体中文](README_zh.md)

基于 [`linkerbot`](https://pypi.org/project/linkerbot/) Python SDK 的 ROS 2
灵巧手驱动。一个节点统一支持 O6、L6、L20Lite、L25、O20、O30I 和 L30。

## 功能

- 通过参数选择手型、左右手、CAN 接口和通信后端
- 发布角度、温度、电流和故障状态
- 接收角度、速度和扭矩命令
- 提供立即读取和故障清除服务
- 对各型号的关节数量、数值范围和 CAN/CAN FD 构造参数进行校验
- 提供 launch、YAML 配置以及可运行的 Python 示例

## 环境要求

- Ubuntu 24.04 + ROS 2 Jazzy
- Python 3.10 或更新版本
- Linkerbot Python SDK 0.5.4 或更新版本
- 已配置的 SocketCAN/CAN FD 接口，或厂商 CAN FD 动态库

先加载 ROS 2 环境，再将 SDK 安装到 ROS 2 使用的 Python 环境：

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install 'linkerbot>=0.5.4'
```

## 构建

```bash
mkdir -p ~/linkerbot_ws/src
cd ~/linkerbot_ws/src
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand-ros.git
cd ..
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/linkerbot_ws/install/setup.bash
```

每次打开新终端都需要加载 ROS 2 和工作空间：

```bash
source /opt/ros/jazzy/setup.bash
source ~/linkerbot_ws/install/setup.bash
```

## 快速开始

以左手 L6 和 `can0` 为例：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left interface_name:=can0
```

新开终端，先读取节点发布的关节顺序：

```bash
ros2 topic echo --once /linkerbot_hand/joint_names
```

将 L6 张开（拇指侧摆使用 SDK 示例值 `50`）：

```bash
ros2 topic pub --once /linkerbot_hand/command/angles \
  std_msgs/msg/Float64MultiArray "{data: [100, 50, 100, 100, 100, 100]}"
```

使用仓库提供的 Python 示例完成同一操作：

```bash
ros2 run linkerbot_hand_ros angle_command --values 100 50 100 100 100 100
```

连续控制时启动常驻 Publisher，随后直接输入多组角度：

```bash
ros2 run linkerbot_hand_ros angle_controller
```

监听关节顺序和实时角度：

```bash
ros2 run linkerbot_hand_ros state_monitor
```

## 文档

- [快速入门与 CAN 配置](docs/zh/getting_started.md)
- [节点参数与型号配置](docs/zh/configuration.md)
- [话题、服务与关节顺序](docs/zh/ros_api.md)
- [Python 与命令行示例](docs/zh/examples.md)
- [故障排查](docs/zh/troubleshooting.md)
- [L6 / ROS 2 Jazzy 实机测试报告](docs/zh/test_reports/l6_jazzy_2026-08-07.md)

## 安全说明

角度命令使用 SDK 的 `0–100` 逻辑范围，但物理方向随型号和关节变化。L6 的屈伸
关节中，`0` 向握拳方向运动，`100` 向张开方向运动；其他型号不能套用这一规则。
连接真实硬件前，请核对对应型号的 SDK 定义、左右手、关节顺序和通信接口，从
非极限角度以及保守的速度、扭矩开始测试。急停和机械限位应独立于本节点部署。
