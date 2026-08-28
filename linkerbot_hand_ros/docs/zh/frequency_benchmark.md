# ROS 角度与完整触觉频率测试

`frequency_benchmark` 只测试两种等工作量场景：

1. percent 角度连续提交 + 角度查询/反馈。
2. percent 角度连续提交 + 角度查询/反馈 + 完整多指触觉事务。

它不会设置速度、电流上限、effort 或扭矩，也不把单个关节、触觉 cell、单指或 CAN 分片
当成独立样本。默认目标取首次有效 percent 角度，`angle_amplitude=0`，因此不会自动全开、
握拳或主动改变姿态。

## 为什么不能只用 `ros2 topic hz`

ROS publish callback 频率、设备采样频率和命令 transport 发送频率不是一个指标。本工具分别
记录：

- 本地 `HandJointCommand` publish Hz 和发送间隔 P95；
- `HandCommandStatus` 中协议终态 Hz、FAILED 和 SUPERSEDED；
- `HandJointState.meta.sequence` 计算的设备角度样本 Hz；
- `HandTactileState.meta.sequence` 计算的完整触觉事务 Hz；
- 起止 `/diagnostics` 中 generation、pending、timeout、route miss 和 bus error 增量。

驱动只在 SDK sequence 或有效性签名变化时发布，工具仍使用 sequence delta 而不是仅统计
ROS 消息数量。

## 测试前提

- 驱动 active，verified capability 和当前 generation 的有效角度已收到。
- 测试期间没有其他角度发布者或主动设备查询程序。
- `command_rate_hz` 启动时不低于最高候选角度频率，并保留调度余量。
- 总线 bitrate、终端电阻、设备 ID、急停和机械空间已单独验证。
- 使用 Hand SDK 1.x 源码或发行包：

```bash
python3 -m pip install \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-hand>=1.0.0.dev0,<2.0.0'
```

启动示例：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left command_rate_hz:=250 \
  transport_json:='{"backend":"socketcan","channel":"can1"}'
```

## 角度单独收发

正式上限测试按每候选点 3 分钟、重复 3 次：

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --angle-rates 50 100 150 200 \
  --duration 180 --repetitions 3
```

工具会对每个候选点原子设置：

- `angles_hz=<候选频率>`
- `force_hz=0`
- `publish_rate_hz=<候选频率>`
- `diagnostic_rate_hz=min(候选频率, 20)`

不会修改静态 `command_rate_hz`，也不会重建连接。

## 角度 + 完整触觉

O6/L6 capability 确认支持完整触觉后，固定触觉频率并搜索角度上限：

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --angle-rates 20 50 100 150 200 \
  --tactile-hz 30 \
  --duration 180 --repetitions 3
```

这里 `30 Hz` 指 30 个完整多指 `HandTactileState`，不是 30 个 CAN frame。若 firmware、
传感器或 capability 的事务成本无法支持，polling plan 可能在参数更新时明确拒绝，或运行时
以 timeout/missed deadline 失败；不得通过提高 ROS publish rate 掩盖。

## 可选小幅扰动

只有确认方向、速度、限制和机械空间后才能启用。以下以当前姿态为中心，索引 2 扰动 3 个
percent 点：

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --angle-rates 50 100 \
  --angle-amplitude 3 --joint-index 2 --angle-period 2 \
  --duration 180 --repetitions 3
```

也可显式给基准目标，数量必须等于 capability 关节数：

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --angle-values 50 50 50 50 50 50 \
  --angle-rates 50 100
```

## 单轮通过标准

默认 `minimum_ratio=0.9`。每轮同时满足：

- 本地命令 publish Hz 不低于候选角度频率的 90%。
- 命令协议成功终态 Hz 不低于 90%。
- 角度 SDK sequence rate 不低于 90%。
- 启用触觉时，完整触觉 sequence rate 不低于触觉目标的 90%。
- 没有 FAILED 或发送前 SUPERSEDED 命令。
- 测试期间 state 和 health 的 connection generation 不变。
- drain 后 `pending_commands=0`。
- consecutive error、角度/force timeout、transport drop/route miss/bus error 无增加。

某候选频率只有连续 3 轮全通过才算稳定。候选集中最高稳定值是本次测试上限。默认首个失败
后停止；需要观察更高点时：

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --angle-rates 50 100 150 200 --continue-after-failure
```

连接断开或 generation 改变会安全中止，恢复后必须重新确认 baseline 并重新运行。

## 不自动改驱动参数

外部系统固定 polling plan 时可使用：

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --no-configure-driver --angle-rates 100
```

此时调用方负责保证 `angles_hz`、`force_hz`、`publish_rate_hz` 和 diagnostics 足以满足目标；
工具仍按消息 sequence 和 health 判据验收。

## 退出码与记录

- `0`：全部候选点的全部轮次通过。
- `2`：正常完成，但至少一个候选点失败。
- `1`：启动、参数、状态或 capability 不满足。
- `130`：用户中断。

每轮同时保存驱动日志、`/diagnostics`、CPU/RSS、线程/FD 数量和
`ip -details -statistics link show canX`。跨 SocketCAN raw、Python SDK 和 ROS 的对比必须
使用相同请求、回复校验、目标更新和测试时长；本 ROS 工具的结果不能与不同工作量的 raw
脚本直接相除。
