# 故障排查

## 找不到 ROS 包

```text
Package 'linkerbot_hand_ros' not found
```

```bash
cd ~/linkerbot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/linkerbot_ws/install/setup.bash
ros2 pkg executables linkerbot_hand_ros
```

本项目只支持当前文档对应的 ROS 2 Jazzy 环境，不要加载 Humble setup。

## Python 找不到手部 SDK

```text
ModuleNotFoundError: No module named 'linkerbot_hand'
```

将拆分后的手部发行包装到运行 ROS 的同一个 Python 环境：

```bash
python3 -c 'import sys; print(sys.executable)'
python3 -m pip install 'linkerbot-hand>=0.1.0.dev1,<0.2.0'
python3 -c 'import linkerbot_hand; print(linkerbot_hand.__file__)'
```

本地 SDK 源码安装：

```bash
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-transport.git
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand.git
python3 -m pip install -e ./linkerbot-transport -e ./linkerbot-hand
```

## 节点存在但没有命令或状态话题

先检查 Lifecycle：

```bash
ros2 lifecycle get /linkerbot_hand
ros2 lifecycle list /linkerbot_hand
```

`ros2 run linkerbot_hand_ros hand_node` 只启动 unconfigured 节点，需要手动执行：

```bash
ros2 lifecycle set /linkerbot_hand configure
ros2 lifecycle set /linkerbot_hand activate
```

使用 `hand.launch.py` 时默认自动激活。inactive 状态会撤销运动命令订阅者并停止 SDK
轮询，因此 `HandCommandClient.is_ready()` 返回 false。

## configure 失败

configure 不访问硬件。失败通常是参数问题：

- `transport_options_json` 不是 JSON 对象。
- JSON 中包含由节点管理的 `model/side/session_config/clock`。
- 型号、side 或频率值无效。
- ROS 环境中的 `linkerbot-hand` 版本不在 0.1.x。

```bash
ros2 param dump /linkerbot_hand
```

## activate 失败或连接为 error

```bash
ip link show can0
ip -details -statistics link show can0
ros2 topic echo --once /diagnostics
```

依次检查型号、左右手、JSON transport 键、SocketCAN/CAN FD 模式、波特率、终端电阻、
设备 ID 和厂商动态库。activate 失败时驱动会关闭候选 backend，不会保留第二个连接。

## 持续通信错误或状态过期

```bash
ros2 param get /linkerbot_hand angles_hz
ros2 param get /linkerbot_hand speeds_hz
ros2 param get /linkerbot_hand torques_hz
ros2 param get /linkerbot_hand current_hz
ros2 param get /linkerbot_hand faults_hz
ros2 param get /linkerbot_hand voltage_hz
ros2 param get /linkerbot_hand acceleration_hz
ros2 param get /linkerbot_hand motion_time_hz
ros2 topic echo --once /diagnostics
```

传感器轮询频率设为 `0` 会禁用对应查询和状态话题。总线负载高时应在线降低相应
SDK 轮询频率，而不是降低 `publish_rate_hz` 来假设总线查询会同步下降。若非零频率
仍无状态，检查当前 SDK capability 是否支持该型号的传感器。诊断中的 `*_age_s`、
`consecutive_errors` 和 `recent_error_code` 用于区分未采样、过期和 transport 错误。

## 状态数组中出现 nan

`NaN` 表示该轴没有独立百分比回读，不是 0，也不是握拳。数组仍保持与
`joint_names` 相同长度。业务到位判断应跳过不支持回读的轴，或使用后续强类型状态
接口中的有效标志。

## 静态参数修改被拒绝

节点 configured、inactive 或 active 时，修改 `model/side/transport_options_json`、旧 transport
参数、`command_rate_hz` 或 `auto_reconnect` 会返回 `successful=false`。这是为了避免参数
显示值与现有 `HandDevice` 不一致。按以下顺序切换：

```bash
ros2 lifecycle set /linkerbot_hand deactivate
ros2 lifecycle set /linkerbot_hand cleanup
ros2 param set /linkerbot_hand model o30i
ros2 lifecycle set /linkerbot_hand configure
ros2 lifecycle set /linkerbot_hand activate
```

## 命令被拒绝

常见原因：

- Lifecycle 不是 active。
- 数组长度不等于 SDK joint count。
- 包含 `NaN/inf` 或超出 capability 范围。
- O20、O30I、L30 的整数型速度或扭矩带小数。
- SDK 正在重连或连接不可用。

```bash
ros2 topic echo --once /linkerbot_hand/joint_names
ros2 topic echo --once /diagnostics
```

## accepted 已发布但实物未动

`command/accepted/*` 只表示 SDK 接收目标。检查诊断中的：

```text
connected
latest_angles_stage
latest_angles_error_code
pending_commands
superseded_commands
```

`accepted` 尚未证明发生 transport I/O；`dispatched` 才表示发送成功；
`acknowledged` 只是协议 ACK。机械到位还要检查新鲜角度反馈和业务超时。

## 单条命令看起来需要约 0.5 秒

`ros2 topic pub --once` 和每次 `ros2 run angle_command` 都会启动新进程并等待 DDS 发现。
使用常驻客户端：

```bash
ros2 run linkerbot_hand_ros angle_controller
```

正式业务中创建一次 `HandCommandClient` 并复用，默认不要在高频循环等待 DDS ACK。

## 没有温度、电流或故障数据

相应型号必须在 SDK capability 中支持传感器，且对应 `temperature_hz/current_hz/faults_hz`
必须大于 0。驱动只发布有效且未过期的缓存；不支持的传感器会自动禁用。

## 清错返回失败

```bash
ros2 service call /linkerbot_hand/clear_faults std_srvs/srv/Trigger '{}'
```

当前 capability 中 L6、L25 和 O20 支持清错。服务成功只表示异步命令进入 SDK，不代表
故障已经消失；仍需观察 fault 状态和命令阶段。不要反复清除仍在发生的过温、过流或
机械卡滞故障。

L30 的 `/enable`、`/disable`、`/calibrate` 同样按 capability 工作；服务成功仅表示 SDK
接受异步操作。其他型号返回不支持不是 ROS 故障。
