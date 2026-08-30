# 故障排查

## 找不到 ROS package

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

不要混用 Humble 和 Jazzy setup。

## 找不到或加载了错误的 Hand SDK

```bash
python3 -c 'import sys, linkerbot_hand; print(sys.executable); print(linkerbot_hand.__file__)'
```

安装 1.x：

```bash
python3 -m pip install \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-hand>=1.0.0.dev0,<2.0.0'
```

开发源码：

```bash
python3 -m pip install -e /path/to/linkerbot-transport -e /path/to/linkerbot-hand
```

旧 0.x 包不提供本驱动需要的 `HandDevice` 1.x 类型契约。

## configure 失败

configure 不连接设备，失败通常是纯配置问题：

- `model` 或 `side` 无效；L30 的 side 必须是空字符串。
- `transport_json` 不是 JSON object、缺 `backend`、缺必填键或含未知键。
- 配置 bitrate 与 capability 冲突。
- queue capacity、timeout、rate 或 reconnect 参数类型/范围错误。
- 请求了 capability 不支持的非零 polling 通道。

```bash
ros2 param dump /linkerbot_hand
```

## activate 失败

activate 才产生 transport open、身份读取和 baseline 事务：

```bash
ip -details -statistics link show can0
ros2 topic echo --once /diagnostics
```

依次检查接口 up 状态、classic CAN/CAN FD 模式、nominal/data bitrate、终端电阻、设备供电、
型号、side、设备地址和 firmware。`unsupported_firmware` 表示设备回复存在，但没有安全匹配的
protocol profile；不能提高 timeout 后继续猜测运行。

activate 失败会停止 polling 并关闭候选连接，可在修复硬件/参数后再次 activate。

## active 但没有命令或状态

```bash
ros2 lifecycle get /linkerbot_hand
ros2 topic list | rg linkerbot_hand
ros2 topic echo --once /linkerbot_hand/capabilities
```

inactive 会撤销命令订阅并停止 polling。状态 Topic 只有 SDK 新 sequence 或有效性变化时才
发布；对应 polling rate 为 0 时不产生消息。

## 状态频率低或过期

```bash
ros2 param get /linkerbot_hand angles_hz
ros2 param get /linkerbot_hand angle_feedback
ros2 param get /linkerbot_hand force_hz
ros2 param get /linkerbot_hand publish_rate_hz
ros2 topic echo --once /diagnostics
```

重点检查 `channel_<name>_requested_hz/achieved_hz/completed_reads/feedback_samples/`
`command_feedback_samples/feedback_policy/missed_deadlines/timeouts/age_s` 以及 transport
frame、route miss 和 bus error。`publish_rate_hz` 只影响无新 revision 时的 age/stale 复查，
不会提高或降低 SDK 查询；新缓存 revision 会立即唤醒发布线程。触觉是完整多帧事务，
角度+触觉上限需用 benchmark 实测。

## 旧固件缺少个别查询

订阅最新 transient-local capability，检查 `firmware_unsupported_telemetry`。SDK 对标为可选
的查询连续 3 次超时或结构不符后，只禁用该语义并保持连接；中间任一次成功会清零失败计数。
ROS 会重新发布 capability，且不再
发布该通道的新样本。重新连接后 SDK 会重新探测。若所有 opcode 都无响应、SocketCAN 只有
TX 没有 RX，应先检查供电、线序、终端和 bitrate，不能误判成某一个协议不支持。

## 百分比或物理角度无效

`HandJointState.raw` 始终保留设备回复。某轴 `percent_valid=false` 表示 capability 没有可信
percent endpoint；`physical_rad_valid=false` 表示没有可信物理标定。不能把占位 0 当成实际
值，也不能把 raw/percent 伪装成弧度。

检查：

```bash
ros2 topic echo --once /linkerbot_hand/capabilities
ros2 topic echo --once /linkerbot_hand/state/angles
```

## 静态参数被拒绝

configured/inactive/active 状态不能修改 model、side、transport、DeviceConfig 或 reconnect：

```bash
ros2 lifecycle set /linkerbot_hand deactivate
ros2 lifecycle set /linkerbot_hand cleanup
ros2 param set /linkerbot_hand model o6
ros2 param set /linkerbot_hand side right
ros2 lifecycle set /linkerbot_hand configure
ros2 lifecycle set /linkerbot_hand activate
```

## 命令被拒绝

常见原因：

- node 非 active 或 SDK 正在恢复。
- `client_id`/`user_sequence`/representation 无效。
- 数组长度不等于 capability channel count。
- NaN、Inf、percent 越界或整数协议值带小数。
- command semantic 不受当前 verified capability 支持。
- 离散 FIFO 已满。

订阅：

```bash
ros2 topic echo /linkerbot_hand/command/status
```

Topic 返回 `HandCommandStatusBatch`；逐条检查 `statuses[]`，按每条状态的 `error_code` 分类，
不解析 `error_message`。

## 已 ACCEPTED 但设备未动

`ACCEPTED` 只表示进入 SDK。检查是否到达 `DISPATCHED`，有 ACK 的型号是否到达
`ACKNOWLEDGED`，以及是否 FAILED/SUPERSEDED。SocketCAN send 成功也不代表设备已机械到位；
到位必须读取当前 generation 的有效、未过期角度。

## 高频命令被大量 SUPERSEDED

连续目标是 latest-wins，这是有意行为。若目标提交高于 SDK/transport 实际发送能力，旧目标
会在发送前被覆盖。检查 `command_rate_hz`、transport pending、CPU/executor backlog 和总线
事务成本。不能通过增大历史 FIFO 保存连续轨迹；轨迹采样应由上层按可实现频率生成。

## 管理服务 timeout

设备信息、堵转读取和 recover 在独立 admin worker 中执行，`timeout_s` 必须为正且小于
`shutdown_timeout_s` 留出的边界。worker FIFO 满返回 `queue_full`；不要用多个客户端并发刷
低频管理事务。

## 断线恢复后旧目标没有执行

这是安全契约。恢复会清除断线前和断线期间目标、重新验证身份并读取 baseline，不会自动
重放。等待新 generation 的有效角度和 fault 后，由业务显式提交新目标。

若返回 `identity_mismatch`，说明重连后的设备指纹不同，必须人工确认，不能继续控制。

## Lifecycle 关闭超时或线程泄漏

检查 `shutdown_timeout_s`、admin service 是否仍被调用、transport backend 是否能有界退出。
不要依赖 Python 析构释放设备，也不要在同一物理接口启动第二个驱动进程。复现时记录：

```bash
ps -L -p <PID>
ls /proc/<PID>/fd | wc -l
ros2 lifecycle get /linkerbot_hand
```

100 次 configure/activate/deactivate/cleanup 后线程、FD 和连接数应回到基线。
