# Linkerbot Hand ROS 2

[English](README.md) | [简体中文](README_zh.md)

面向正式业务部署的 Linkerbot 灵巧手 ROS 2 Jazzy 驱动。一个 Lifecycle 节点统一支持
O6、L6、L20Lite、L25、O20、O30I 和 L30，型号能力、轮询、命令调度和断线恢复均由
独立的 `linkerbot-hand` Python SDK 管理。

## 运行边界

```text
业务节点 / HandCommandClient
          | ROS 2 Topic、Service、QoS、Lifecycle
          v
linkerbot_hand（每只手唯一驱动节点）
          | HandDevice、PollingPlan、CommandHandle
          v
SDK 单 I/O worker -> transport -> 灵巧手
```

- `on_configure` 只创建设备配置，不连接、不查询、不运动。
- `on_activate` 打开唯一连接并启动 SDK 轮询。
- ROS 发布定时器只读取 SDK 缓存，不产生设备 I/O。
- 连续命令采用 latest-wins，回调不等待 transport 或设备 ACK。
- `on_deactivate` 停止轮询，`on_cleanup/on_shutdown` 关闭连接。
- 屈伸轴统一使用 `100=完全张开，0=完全握拳`；侧摆、旋转等方向轴的 0/100
  表示 capability 定义的两个位置端点，不能解释成张开/握拳。
- 另行提供协议 `raw_angles` 命令与缓存状态；该坐标绕过 classic 用户 LUT。

## 环境要求

- Ubuntu 24.04 + ROS 2 Jazzy
- Python 3.10 或更新版本
- `linkerbot-hand>=0.1.2,<0.2.0`
- 已配置的 SocketCAN/CAN FD 接口，或型号支持的厂商动态库

`linkerbot-hand v0.1.2` 已包含当前驱动所需的物理单位契约，并兼容 ROS 2 Jazzy 的
NumPy 1.x 环境。

必须使用 Jazzy：

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-hand>=0.1.2,<0.2.0'
```

本地开发 SDK 时，可从两个独立仓库安装。`linkerbot-hand` 声明兼容的
`linkerbot-transport` 依赖，ROS 包不直接依赖 transport：

```bash
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-transport.git
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand.git
python3 -m pip install -e ./linkerbot-transport -e ./linkerbot-hand
python3 -c 'import linkerbot_hand; print(linkerbot_hand.__file__)'
```

## 构建

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

Ubuntu 24.04 的 `/usr/bin/rosdep` 可能输出 `pkg_resources is deprecated`，这是系统
rosdep 启动脚本的弃用警告，不影响依赖解析；但任何 `ERROR:` 都代表真实失败，不能忽略。

每个新终端都需要：

```bash
source /opt/ros/jazzy/setup.bash
source ~/linkerbot_ws/install/setup.bash
```

## 启动 L6

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left \
  transport_options_json:='{"interface_name":"can0","interface_type":"socketcan"}'
```

launch 默认 `autostart:=true`，会依次 configure 和 activate：

```bash
ros2 lifecycle get /linkerbot_hand
ros2 topic echo --once /linkerbot_hand/joint_names
ros2 topic echo --once /linkerbot_hand/state/angles
```

启动后可在线调整或关闭轮询与状态上报，不会重建 CAN 连接：

```bash
ros2 param set /linkerbot_hand torques_hz 30.0
ros2 param set /linkerbot_hand torques_hz 0.0
ros2 param set /linkerbot_hand publish_rate_hz 0.0
ros2 param set /linkerbot_hand publish_rate_hz 60.0
```

`model`、`side`、transport、命令频率和重连策略只允许在 unconfigured 状态修改；
已配置后必须先 deactivate 和 cleanup，避免参数显示值与实际连接不一致。

连续控制应使用常驻客户端，避免每条命令重新启动进程并等待 DDS 发现：

```bash
ros2 run linkerbot_hand_ros angle_controller
```

确认速度、扭矩和机械环境安全后，L6 完全张开目标为：

```text
100 100 100 100 100 100
```

完全握拳目标为：

```text
0 0 0 0 0 0
```

## 文档

- [快速入门与 CAN 配置](docs/zh/getting_started.md)
- [Lifecycle、参数与型号配置](docs/zh/configuration.md)
- [话题、服务、诊断与关节顺序](docs/zh/ros_api.md)
- [正式业务节点集成](docs/zh/business_integration.md)
- [命令行与 Python 示例](docs/zh/examples.md)
- [50–100 Hz 收发与频率上限测试](docs/zh/frequency_benchmark.md)
- [故障排查](docs/zh/troubleshooting.md)
- [从 0.2.x 迁移到 0.3.0](docs/zh/migration_0_3.md)

## 安全说明

屈伸轴的 `100/0` 是统一的张开度目标，方向轴的 `100/0` 是位置端点；这些值不代表
弧度，也不代表所有轴都有可观测的百分比回读。
首次连接实物时应先验证型号、左右手、关节顺序、总线和急停，从中间值、低速度和保守
扭矩开始。DDS ACK、SDK `ACCEPTED`、设备协议 ACK 和机械到位是四种不同状态，业务
安全逻辑必须根据有效且未过期的状态反馈判断到位。
