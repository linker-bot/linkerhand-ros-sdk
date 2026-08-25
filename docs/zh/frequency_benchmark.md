# 关节命令与状态频率测试

本工具用于回答两个不同问题：

1. ROS 业务节点能否以 50–100 Hz 持续提交角度目标，SDK 是否及时 dispatch，队列是否保持
   有界。
2. 角度和力矩/电流状态是否真的产生了 50–100 Hz 的新设备样本，而不是 ROS 重复发布同一
   份 SDK 缓存。

只运行 `ros2 topic hz` 不能回答第二个问题。`state/*` 使用 `Float64MultiArray`，不包含采集
序号；当 `publish_rate_hz` 高于 SDK polling 频率时，同一缓存可能被多次发布。基准程序同时
读取 `/diagnostics` 中的 `<sensor>_sequence` 和连接代次，以序号增量计算真实采样频率。

## 测试前提

- Ubuntu 24.04、ROS 2 Jazzy，驱动处于 Lifecycle `active`。
- CAN/CAN FD 接口、型号、左右手、关节顺序和急停已经单独验证。
- 测试期间停止其他角度、速度和力矩命令发布者，否则 accepted 和覆盖计数无法归因。
- `command_rate_hz` 是静态参数，启动时应大于等于最高候选命令频率，并留出调度余量。
- 必须安装 `linkerbot-hand>=0.1.2,<0.2.0`，该版本包含当前速度、电流和电流限制物理单位
  契约。

```bash
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-transport.git
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand.git
python3 -m pip install -e ./linkerbot-transport -e ./linkerbot-hand
python3 -c 'import linkerbot_hand; print(linkerbot_hand.__file__)'

source /opt/ros/jazzy/setup.bash
source ~/linkerbot_ws/install/setup.bash
```

以 L6、最高候选 100 Hz 为例，先启动驱动：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left command_rate_hz:=150 \
  transport_options_json:='{"interface_name":"can0","interface_type":"socketcan"}'
```

基准程序会在线同步 `angles_hz`、力矩或电流反馈 polling、`publish_rate_hz` 和
`diagnostic_rate`，不会重建连接。它不会修改静态的 `command_rate_hz`。

## 推荐测试：力矩限制一次，角度连续发送

`control` 是正常控制路径：先把力矩/电流限制提交一次，并等待诊断确认其已
`dispatched` 或 `acknowledged`；随后才连续发送角度。角度目标默认取首次收到的当前姿态，
扰动幅度默认为 0，因此不会自动全开、握拳或主动改变姿态。

必须根据型号和实物风险评估显式填写限制。以下 `$LIMIT` 不是固定推荐值：

```bash
LIMIT=<已确认安全的该型号限制值>
ros2 run linkerbot_hand_ros frequency_benchmark \
  --mode control \
  --torque-value "$LIMIT" \
  --rates 50 60 80 100 \
  --duration 30 \
  --repetitions 3
```

也可以逐关节填写，数量和顺序必须与 `/linkerbot_hand/joint_names` 一致：

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --mode control \
  --torque-values <J1> <J2> <J3> <J4> <J5> <J6>
```

工具默认同时要求角度和负载反馈达到候选频率。不同型号的“负载反馈”含义如下：

| 型号 | 设置话题 `command/torques` | 第二反馈 | capability 推荐频率 |
| --- | --- | --- | --- |
| L6 | 电流限制，mA | `state/torques`，mA | 角度 60 Hz，负载 30 Hz |
| O6 | 转矩限制，mA | `state/torques`，mA | 角度 60 Hz，负载 30 Hz |
| L20Lite、L25 | 无量纲限制 | `state/torques`，无量纲 | 角度 60 Hz，负载 30 Hz |
| O30I | `0–255` 协议值 | `state/torques`，协议值 | 角度 30 Hz，负载 30 Hz |
| O20 | 目标限制，`0–1000` 整数，每单位 6.5 mA | `state/current`，mA | 角度 30 Hz，电流 10 Hz |
| L30 | 电流上限，mA | `state/current`，mA | 角度 60 Hz，电流 10 Hz |

O20、L30 的电流反馈不能描述成机械关节扭矩。推荐频率不是硬上限，但 50–100 Hz 组合测试
很可能先受负载反馈 polling 限制；失败结果应保留，不能通过提高 ROS 发布频率掩盖。

只评估角度命令和真实角度反馈时使用：

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --mode angles --feedback angles \
  --rates 50 60 80 100 --duration 30 --repetitions 3
```

## 可选小幅运动验证

只有在机械空间、方向、速度和限制值都确认后，才启用单关节正弦扰动。以下命令以当前姿态
为中心，仅扰动索引 2，幅度 3 个百分比点：

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --mode control --torque-value "$LIMIT" \
  --angle-amplitude 3 --joint-index 2 --angle-period 2 \
  --rates 50 60 80 100 --duration 30 --repetitions 3
```

若当前 `state/angles` 含 `NaN`，工具不会猜测目标，而会停止并要求显式传入
`--angle-values`。显式目标可以继续做命令压力测试，但严格的“全部关节角度状态有限”判据
仍会失败并报告原因。方向轴的 0/100 是 capability 定义的位置端点，不应解释为握拳/张开。

## 其他压力模式

| 模式 | 行为 | 用途 |
| --- | --- | --- |
| `control` | 力矩/电流限制一次，角度连续发送 | 推荐的正常控制测试 |
| `angles` | 只连续发送角度 | 隔离角度命令上限 |
| `torques` | 连续发送限制值 | 非正常业务路径，仅用于协议压力测试 |
| `mixed` | 每个 tick 在角度和限制值之间交替 | 验证共享 latest-wins 槽；候选频率是两类命令合计值 |

连续运动命令共用 SDK 容量为 1 的 latest-wins 槽。因此，不能把“角度 100 Hz + 力矩
100 Hz”理解成两个互不影响的 100 Hz 队列。`mixed --rates 100` 表示总计 100 Hz，每类约
50 Hz。正常控制应使用 `control`，而不是高频重复设置相同限制。

## 通过标准

每个候选频率默认持续 30 秒并重复 3 次，单轮同时满足以下条件才通过：

- 实际 ROS publish 频率不低于目标的 95%。
- `command/accepted/*` 数量不低于 publish 数的 95%。
- 没有命令校验或 publish 异常；程序同时报告本地错过周期数和发送间隔 P95。
- `superseded_commands` 增量不超过发送数的 1%。
- 排空等待后 `pending_commands == 0`。
- `ros_commands_rejected` 和 `ros_accepted_feedback_errors` 增量为 0。
- 测试期间 `connection_generation` 不变，结束时设备已连接且无连续通信错误。
- 最新命令终态为 `dispatched` 或 `acknowledged`；`accepted` 不算已发送。
- 角度及选定负载状态消息长度正确且全部为有限值。
- ROS 状态 topic 频率不低于目标的 90%。
- 同一连接代次内，SDK 采样序号计算出的真实设备频率不低于目标的 90%。

某一候选频率只有连续 3 轮全部通过，才计为稳定。候选集中的最高连续通过值就是本次测试
上限；首个失败频率和失败原因必须记录。默认在首个失败频率后停止，可用
`--continue-after-failure` 收集后续点：

连接中断或代次变化属于安全中止条件。程序会立即停止提交后续目标，即使指定了
`--continue-after-failure` 也不会继续下一轮，恢复后必须重新确认当前状态并重新启动测试。

```bash
ros2 run linkerbot_hand_ros frequency_benchmark \
  --mode control --torque-value "$LIMIT" \
  --rates 50 60 80 100 120 150 200 \
  --duration 30 --repetitions 3 \
  --continue-after-failure
```

单次偶发峰值不是上限。提高频率时还应同步保存 `ip -details -statistics link show can0`、
驱动日志和 `/diagnostics`，区分 Python 调度、SDK 覆盖、总线负载、设备响应和断线恢复问题。

## 退出码

- `0`：所有候选频率均连续通过。
- `2`：程序正常完成，但至少一个候选频率未通过。
- `1`：启动、参数、设备状态或测试配置错误。
- `130`：用户中断。
