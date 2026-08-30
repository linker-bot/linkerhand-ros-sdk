# 快速入门

## 1. 环境

本文只适用于 Ubuntu 24.04、ROS 2 Jazzy 和 Hand SDK 1.x。每个终端先执行：

```bash
source /opt/ros/jazzy/setup.bash
```

安装已发布 SDK：

```bash
python3 -m pip install \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-hand>=1.0.0.dev0,<2.0.0'
```

测试尚未发行的源码时，在同一个 Python 环境安装：

```bash
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-transport.git
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand.git
python3 -m pip install -e ./linkerbot-transport -e ./linkerbot-hand
python3 -c 'import linkerbot_hand; print(linkerbot_hand.__file__)'
```

系统工具：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep can-utils
```

## 2. CAN 接口

经典 CAN 示例：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

CAN FD 示例：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
ip -details -statistics link show can0
```

实际 nominal/data bitrate 必须与设备 capability 和硬件手册一致。不要在驱动运行时再启动
另一个会主动查询设备的测试程序；`candump` 只监听，可用于检查总线。

## 3. 构建 ROS 工作空间

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

rosdep 的弃用警告可以记录后继续，unresolved key 或 `ERROR:` 必须处理。

## 4. 启动与检查

L6 左手：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left \
  transport_json:='{"backend":"socketcan","channel":"can0"}'
```

L30 不区分左右手：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l30 side:='' \
  transport_json:='{"backend":"socketcan","channel":"can0"}'
```

L6/O6 等多手部署使用同一进程：

```bash
ros2 launch linkerbot_hand_ros multi_hand.launch.py
```

默认读取随包安装的 `config/multi_hand.yaml`。使用自己的绝对路径：

```bash
ros2 launch linkerbot_hand_ros multi_hand.launch.py \
  config:=/absolute/path/to/hands.yaml
```

参数文件中的 `device_namespaces` 决定内部 Lifecycle 节点。每个 namespace 必须有对应的
`/<namespace>/linkerbot_hand` 参数段。相同物理 `transport_json` 的设备在本进程内共享总线
runtime；不同 channel 分别建立 runtime。不要用两个独立 launch 进程代替该入口。

确认 active、身份已验证且角度属于当前连接代次：

```bash
ros2 lifecycle get /linkerbot_hand
ros2 topic echo --once /linkerbot_hand/capabilities
ros2 topic echo --once /linkerbot_hand/device_info
ros2 topic echo --once /linkerbot_hand/state/angles
ros2 topic echo --once /diagnostics
```

`capabilities.identity.verified` 应为 `true`。`state/angles.meta` 应满足 `valid=true`、
`stale=false`，且 `connection_generation` 与 diagnostics 一致。

## 5. 无运动通信验证

先只观察状态，不发送目标：

```bash
ros2 topic hz /linkerbot_hand/state/angles
ros2 param set /linkerbot_hand angles_hz 60.0
ros2 param set /linkerbot_hand angles_hz 0.0
```

`angles_hz=0` 后 SDK 不再产生角度查询。恢复轮询：

```bash
ros2 param set /linkerbot_hand angles_hz 60.0
```

## 6. 首条命令

先从当前姿态附近的小范围目标开始。以下只展示消息格式，不代表任意设备都可安全直接运动：

```bash
ros2 topic pub --once /linkerbot_hand/command/angles \
  linkerbot_hand_msgs/msg/HandJointCommand \
  '{client_id: 1, user_sequence: 1, representation: 1, values: [50, 50, 50, 50, 50, 50]}'
ros2 topic echo /linkerbot_hand/command/status
```

`representation=1` 是 percent；`representation=2` 是协议 raw。关节数量和顺序只能从
capability 读取。收到 `DISPATCHED` 只证明 transport send 成功，机械到位仍需状态反馈判断。
`/command/status` 的类型是 `HandCommandStatusBatch`；其中每个 `statuses[]` 元素对应一条命令，
不能把整批消息当成单条状态。

下一步参阅[参数配置](configuration.md)、[ROS API](ros_api.md)和
[业务集成](business_integration.md)。
