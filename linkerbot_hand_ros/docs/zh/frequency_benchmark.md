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
- 当前 `client_id` 的驱动收到 Hz、SDK `ACCEPTED` Hz、实际 `DISPATCHED` Hz 和协议终态 Hz；
- 发布到驱动接收、接收到 SDK 接受、接受到实际下发及发布到实际下发的 P50/P95/P99；
- `HandCommandStatus.transitions[]` 中的 FAILED 和 SUPERSEDED；
- SDK diagnostics 中 `completed_reads` 计算的完整读取事务 Hz；
- SDK diagnostics 中角度 `command_feedback_samples`、`feedback_samples` 与
  `feedback_policy`，用于区分命令反馈和真正计入反馈频率的样本；
- ROS 状态消息 `meta.sequence` 增量计算的 SDK 缓存更新 Hz；
- 实际收到的有效 ROS 状态消息计算的 ROS publish Hz；
- 起止 `/diagnostics` 中 generation、pending、timeout、route miss 和 bus error 增量。

这三个状态频率不能互相替代。查询响应、设置回显和主动上报都可能增加 cache sequence，只有
`completed_reads` 表示 polling plan 下完成的逻辑读取；一次完整多帧触觉事务只增加一次。
ROS publish Hz 则单独反映状态发布和 DDS 接收链路。diagnostics 同时提供最新 `origin`，用于
确认当前缓存来自查询响应、设备上报还是回显。命令阶段按
`(client_id, user_sequence, stage)` 去重；最终状态为 `ACKNOWLEDGED` 时仍以完整历史中是否
存在 `DISPATCHED` 为准。

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
- 驱动收到命令 Hz 不低于 90%。
- SDK `ACCEPTED` Hz 不低于 90%。
- 命令协议终态 Hz 不低于 90%。
- 完整阶段历史中实际进入 `DISPATCHED` 的命令 Hz 不低于 90%。
- 角度读取完成 Hz、缓存更新 Hz 和 ROS publish Hz 分别不低于角度目标的 90%。
- 启用触觉时，完整触觉读取完成 Hz、缓存更新 Hz 和 ROS publish Hz 分别不低于触觉目标的
  90%。
- 没有 FAILED 命令；SUPERSEDED 单独记录，并通过实际下发频率门槛约束。
- 四段延迟时间戳没有缺失或倒序。
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
