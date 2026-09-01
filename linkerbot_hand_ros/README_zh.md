# Linkerbot Hand ROS 2

[English](README.md) | [简体中文](README_zh.md)

面向 ROS 2 Jazzy 的 Linkerbot 灵巧手 Lifecycle 驱动，统一支持 O6、L6、L20Lite、
L25、O20、O30I 和 L30。ROS 只使用 `linkerbot-hand` 1.x 的唯一公开设备类
`HandDevice`，型号、关节顺序、范围和传感器能力全部来自 SDK capability。

## 运行边界

```text
业务节点 / HandCommandClient
          | 类型化 Topic、Service、QoS、Lifecycle
          v
linkerbot_hand LifecycleNode
          | 唯一 HandDevice
          v
SDK polling / scheduler -> linkerbot-transport -> 设备
```

- configure 只构造配置、transport 描述和 ROS 实体，不连接、不查询、不运动。
- activate 才调用 `open()`，验证身份和固件，并启动 SDK polling。
- 命令 callback 只调用非阻塞 `set_*()`，不等待 transport 或设备 ACK。
- 状态定时器只读取 `get_snapshot()`，同一 SDK sequence 只发布一次。
- diagnostics 只读取 `get_health()`，不触发故障查询。
- deactivate 撤销命令入口并停止 polling，但保留连接；cleanup/shutdown 有界关闭。
- ROS 不维护型号表、不创建第二套查询定时器，也不提供旧数组兼容话题。

角度的协议 raw、percent 和可选物理弧度分别保存。屈伸轴固定为
`0=弯曲/闭合，100=伸直/张开`；侧摆、旋转等方向轴的 0/100 含义读取
`HandCapabilities.joints[].percent_mapping`，不能称为张开/握拳。

旧固件缺少个别可选查询时，SDK 保持连接并只禁用该通道；ROS 会重新发布
`HandCapabilities.firmware_unsupported_telemetry`。O6/L6 支持的配置持久化通过
`/linkerbot_hand/save_parameters` 服务提交，并以 command status 的协议 ACK 判断成功。

## 环境与安装

- Ubuntu 24.04 + ROS 2 Jazzy
- Python 3.10 或更新版本
- `linkerbot-transport>=1.0.0,<2.0.0`
- `linkerbot-hand>=1.0.0,<2.0.0`
- 已配置的 SocketCAN/CAN FD 接口或受支持厂商适配器

首次安装可以直接执行[仓库快速启动](../README_zh.md#快速启动)。手工分步安装时，在能够
读取 ROS 系统包的隔离环境中显式安装 Transport 和 Hand：

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages ~/venvs/linkerbot-hand-ros-1.0.0
source ~/venvs/linkerbot-hand-ros-1.0.0/bin/activate
python -m pip install \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-transport[all]==1.0.0' \
  'linkerbot-hand==1.0.0'
```

测试尚未发行的 1.x 源码时，在 ROS 使用的同一个 Python 环境安装独立仓库：

```bash
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-transport.git
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand.git
python -m pip install -e ./linkerbot-transport -e ./linkerbot-hand
python -c 'import linkerbot_hand; print(linkerbot_hand.__file__)'
```

## 构建

`linkerbot_hand_msgs` 包含 ROSIDL 接口，必须针对当前 ROS 发行版生成，不能放入
与平台无关的驱动 wheel。因此 ROS 的正式交付以同一个 `v1.0.0` 标签下的两个源码
package 为准：

```bash
mkdir -p ~/linkerbot_ws/src
cd ~/linkerbot_ws/src
git clone --branch v1.0.0 --depth 1 \
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

系统 rosdep 输出的 `pkg_resources is deprecated` 是警告；任何 unresolved key 或
`ERROR:` 都是实际失败，不能忽略。
以上验证只检查依赖、ROSIDL 接口、ament 索引和程序入口，不连接设备或发送 CAN 帧。

## 启动

L6 左手、SocketCAN `can0`：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left \
  transport_json:='{"backend":"socketcan","channel":"can0"}'
```

L30 不区分左右手，必须传空 side：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l30 side:='' \
  transport_json:='{"backend":"socketcan","channel":"can0"}'
```

多手设备必须使用同进程入口：

```bash
ros2 launch linkerbot_hand_ros multi_hand.launch.py
```

随包安装的 `config/multi_hand.yaml` 会创建 `/l6_left/linkerbot_hand` 和
`/o6_right/linkerbot_hand`。正式部署通过
`config:=/absolute/path/to/hands.yaml` 指定自己的参数文件。只有
`transport_json` 产生相同物理 key 的设备才共享一个 transport runtime、RX reactor 和 TX
arbiter；启动两次 `hand.launch.py` 会产生两个进程，不能提供进程内总线仲裁。

launch 默认 `autostart:=true`。检查 verified capability、设备信息和角度：

```bash
ros2 lifecycle get /linkerbot_hand
ros2 topic echo --once /linkerbot_hand/capabilities
ros2 topic echo --once /linkerbot_hand/device_info
ros2 topic echo --once /linkerbot_hand/state/angles
ros2 topic echo /linkerbot_hand/command/status
```

## 发送角度

类型化 percent 命令，数组顺序取 capability 的 `joints[].name`：

```bash
ros2 topic pub --once /linkerbot_hand/command/angles \
  linkerbot_hand_msgs/msg/HandJointCommand \
  '{client_id: 1, user_sequence: 1, representation: 1, values: [50, 50, 50, 50, 50, 50]}'
```

- `representation=1`：percent，严格 `0..100`。
- `representation=2`：精确协议 raw，不做方向、LUT 或物理单位转换。
- SDK 接受后通过 `/command/status` 的 `HandCommandStatusBatch.statuses[]` 发布
  `ACCEPTED`；实际发送后才是 `DISPATCHED`。只有设备协议明确确认时才会出现
  `ACKNOWLEDGED`。
- 正式程序使用 `HandCommandClient` 自动生成非零 `client_id`，并以
  `(client_id, user_sequence)` 关联状态；多个发布者不能只按序号匹配。

高频控制复用常驻客户端：

```bash
ros2 run linkerbot_hand_ros angle_controller
```

## 在线频率配置

只有 polling 和 ROS 发布频率可在 active 状态原子修改：

```bash
ros2 param set /linkerbot_hand angles_hz 100.0
ros2 param set /linkerbot_hand angle_feedback query_only
ros2 param set /linkerbot_hand force_hz 30.0
ros2 param set /linkerbot_hand publish_rate_hz 100.0
ros2 param set /linkerbot_hand diagnostic_rate_hz 10.0
```

通道频率设为 0 后 SDK 不再产生该类查询。`publish_rate_hz=0` 只停止 ROS 状态发布，
不会替代 `angles_hz=0`。型号、side、transport 和 runtime 容量等静态参数需先
`deactivate -> cleanup` 才能修改。

SDK 缓存 revision 变化会立即唤醒独立状态发布线程，不再依赖与设备反馈同频的 timer
扫描。`publish_rate_hz` 控制无新数据时 age/stale 的缓存复查频率，不限制新 SDK sequence
的即时发布频率；未变化的 sequence 不会重复发布。

`angle_feedback` 可选 `query_only`、`command_feedback_only` 或 `adaptive`。主动设置并
主动查询的等工作量测试必须使用 `query_only`；只用设备设置回显做数采时使用
`command_feedback_only`；生产默认 `adaptive`，由回显满足反馈目标并只查询缺口。
`query_only` 下回显仍保留在 SDK 缓存和指标中，但不会发布到 ROS 角度状态 topic。

## 文档

- [快速入门](docs/zh/getting_started.md)
- [Lifecycle 与参数](docs/zh/configuration.md)
- [ROS API reference](docs/zh/ros_api.md)
- [业务集成](docs/zh/business_integration.md)
- [命令与 Python 示例](docs/zh/examples.md)
- [频率上限测试](docs/zh/frequency_benchmark.md)
- [故障排查](docs/zh/troubleshooting.md)
- [版本变更记录](CHANGELOG.rst)

所有影响驱动行为、配置、依赖、安装工具或用户文档的 PR，都必须同步更新
`CHANGELOG.rst` 的“Unreleased”章节。消息或服务契约变更还必须同步更新
`linkerbot_hand_msgs/CHANGELOG.rst`。

## 安全说明

DDS 发布成功、SDK `ACCEPTED`、transport `DISPATCHED`、设备协议 ACK 和机械到位不可
互相替代。机械到位必须使用当前 `connection_generation` 内有效、未过期的状态反馈判断。
恢复后 SDK 不重放断线前目标，业务层必须重新确认状态并显式下发新目标。
