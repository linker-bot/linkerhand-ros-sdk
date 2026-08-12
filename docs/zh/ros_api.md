# ROS 2 接口

默认节点名为 `/linkerbot_hand`。源码使用 `~/` 私有名称，namespace 会自动加入所有
私有接口。状态发布器和运动命令订阅者只在 Lifecycle `active` 状态工作。

## 角度约定

所有型号对外统一为：

```text
100 = 完全张开
0   = 完全握拳
```

该值是百分比，不是弧度。SDK capability 是关节顺序、范围、整数约束、传感器能力和
原始值映射的唯一数据源，ROS 不维护第二份型号表。

某些握姿插值轴没有独立百分比回读。为了保持数组长度和 `joint_names` 一致，
`state/angles` 对这些位置发布 `NaN`。业务到位判断必须拒绝 `NaN`，并且不能把百分比
直接写入 `sensor_msgs/JointState.position`。

## 话题

| 完整话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/linkerbot_hand/command/angles` | `std_msgs/msg/Float64MultiArray` | 订阅 | 张开度目标，0–100 |
| `/linkerbot_hand/command/speeds` | `std_msgs/msg/Float64MultiArray` | 订阅 | SDK capability 定义的速度目标 |
| `/linkerbot_hand/command/torques` | `std_msgs/msg/Float64MultiArray` | 订阅 | SDK capability 定义的扭矩目标 |
| `/linkerbot_hand/command/accepted/angles` | `std_msgs/msg/Float64MultiArray` | 发布 | SDK 已接受的归一化角度目标 |
| `/linkerbot_hand/command/accepted/speeds` | `std_msgs/msg/Float64MultiArray` | 发布 | SDK 已接受的归一化速度目标 |
| `/linkerbot_hand/command/accepted/torques` | `std_msgs/msg/Float64MultiArray` | 发布 | SDK 已接受的归一化扭矩目标 |
| `/linkerbot_hand/state/angles` | `std_msgs/msg/Float64MultiArray` | 发布 | 有效且未过期的缓存角度百分比 |
| `/linkerbot_hand/state/speeds` | `std_msgs/msg/Float64MultiArray` | 发布 | 型号支持时发布有效速度缓存 |
| `/linkerbot_hand/state/torques` | `std_msgs/msg/Float64MultiArray` | 发布 | 型号支持时发布有效力矩缓存 |
| `/linkerbot_hand/state/temperature` | `std_msgs/msg/Float64MultiArray` | 发布 | 有效且未过期的缓存温度 |
| `/linkerbot_hand/state/current` | `std_msgs/msg/Float64MultiArray` | 发布 | 有效且未过期的缓存电流 |
| `/linkerbot_hand/state/fault` | `std_msgs/msg/Float64MultiArray` | 发布 | 有效且未过期的缓存故障值 |
| `/linkerbot_hand/joint_names` | `std_msgs/msg/String` | 发布 | 逗号分隔的 SDK 关节顺序 |
| `/linkerbot_hand/model` | `std_msgs/msg/String` | 发布 | 小写规范化型号名 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 发布 | SDK 健康、缓存年龄和命令阶段 |

数组必须严格按照 `joint_names` 的顺序和数量。长度、非有限值、范围或整数约束不满足
时，驱动拒绝命令，不会提交给 SDK。

状态定时器只调用 `get_snapshot()`；无效或 stale 的样本不会发布。设备查询频率只由
SDK `PollingPlan` 决定。轮询频率参数可在 active 状态动态修改，设为 `0` 后立即屏蔽
对应状态话题并停止 SDK 查询，不重建设备连接。完整命令见[节点配置](configuration.md)。

## QoS 与 latest-wins

运动命令使用 `RELIABLE + KEEP_LAST(depth=1) + VOLATILE`。ROS executor 只保留最新
待处理目标，SDK 连续命令槽也使用 `CommandPolicy.LATEST`，旧目标不会排队让机械手
逐条追赶。

元数据和 `command/accepted/*` 使用
`RELIABLE + KEEP_LAST(depth=1) + TRANSIENT_LOCAL`，后启动的业务节点能获取最近值。

## 命令确认语义

`command/accepted/*` 只表示参数有效并进入 SDK 调度器，即 SDK `CommandStage.ACCEPTED`。
它不表示已经发出 CAN 帧，不表示设备 ACK，更不表示机械到位。

| 阶段 | 含义 |
| --- | --- |
| `accepted` | 已进入 SDK 调度器，尚不保证 transport I/O |
| `dispatched` | transport 实际发送成功；不支持设备 ACK 时为成功终态 |
| `acknowledged` | 协议明确返回 ACK；不是机械到位 |
| `failed` | 发送、通信或设备确认失败 |
| `superseded` | 发送前被更新目标覆盖，符合 latest-wins |

最近三类连续命令的 ID、stage 和错误码位于 `/diagnostics`：

```text
latest_angles_command_id / latest_angles_stage / latest_angles_error_code
latest_speeds_command_id / latest_speeds_stage / latest_speeds_error_code
latest_torques_command_id / latest_torques_stage / latest_torques_error_code
latest_clear_faults_command_id / latest_clear_faults_stage / latest_clear_faults_error_code
```

O30I 的连续命令支持设备协议 ACK。L6 等不支持 ACK 的型号以 `dispatched` 为成功终态。
机械到位必须由业务层使用同一 `connection_generation` 内有效且未过期的状态、目标
误差和稳定时间判断。

## 服务

| 服务 | 类型 | 语义 |
| --- | --- | --- |
| `/linkerbot_hand/read_now` | `std_srvs/srv/Trigger` | 立即发布当前新鲜角度缓存，不产生设备 I/O |
| `/linkerbot_hand/clear_faults` | `std_srvs/srv/Trigger` | 提交异步清错命令，成功仅表示 SDK 接受 |

```bash
ros2 service call /linkerbot_hand/read_now std_srvs/srv/Trigger '{}'
ros2 service call /linkerbot_hand/clear_faults std_srvs/srv/Trigger '{}'
```

当前 SDK capability 中 L6、L25 和 O20 支持清错；其他型号会明确返回
`success=false`。inactive 状态下两个服务都不会操作设备。

Lifecycle 节点同时提供标准的 `change_state`、`get_state`、
`get_available_states` 和 `get_available_transitions` 服务。

## 诊断

`/diagnostics` 的级别映射：

| 条件 | 级别 |
| --- | --- |
| inactive | `WARN` |
| active、已连接、角度缓存有效且未过期 | `OK` |
| 正在重连、等待首个样本、缓存过期、单次 SDK 错误 | `WARN` |
| 最新 ROS 状态转换、命令校验或 accepted 反馈发布失败 | `WARN` |
| active 但连接为 closed/error，或连续错误达到阈值 | `ERROR` |

关键字段包括 `connection_state`、`connected`、`connection_generation`、
`reconnect_count`、`consecutive_errors`、`pending_commands`、
`superseded_commands`、各传感器 `*_age_s`、SDK 稳定错误码和最近命令阶段。
`ros_accepted_feedback_errors` 单独统计 SDK 已接受命令之后的 ROS 反馈发布失败，
不会把可能已经执行的命令误记为 rejected。

## 命令范围

以下为当前 SDK 0.1.x capability 摘要，运行时仍以实际 capability 为准：

| 型号 | 角度 | 速度 | 扭矩 |
| --- | --- | --- | --- |
| O6、L6、L20Lite、L25 | 0–100 | 0–100 | 0–100 |
| O20 | 0–100 | 0–100 整数 | 0–1000 整数 |
| O30I | 0–100 | 0–255 整数 | 0–255 整数 |
| L30 | 0–100 | 1–250 整数 | 60–800 整数 |

ROS 话题类型为 Float64 数组；驱动会把值为整数的 Float64 转成 SDK 所需整数，带小数
的整数型命令会被拒绝。

## 各型号关节顺序

### O6 / L6（6）

`thumb_flex, thumb_abd, index, middle, ring, pinky`

### L20Lite（10）

`thumb_flex, thumb_abd, index_flex, middle_flex, ring_flex, pinky_flex,`
`index_abd, ring_abd, pinky_abd, thumb_yaw`

### L25（16）

`thumb_abd, thumb_yaw, thumb_root1, thumb_tip, index_abd, index_root1,`
`index_tip, middle_abd, middle_root1, middle_tip, ring_abd, ring_root1,`
`ring_tip, pinky_abd, pinky_root1, pinky_tip`

### O20（16）

`thumb_mcp, thumb_ip, thumb_abd, thumb_cmc, index_abd, index_mcp, index_pip,`
`middle_abd, middle_mcp, middle_pip, ring_abd, ring_mcp, ring_pip, pinky_abd,`
`pinky_mcp, pinky_dip`

### O30I（20）

`roll_0, yaw_0, yaw_1, yaw_2, yaw_3, yaw_4, root1_0, root1_1, root1_2,`
`root1_3, root1_4, root2_1, root2_2, root2_3, root2_4, tip_0, tip_1, tip_2,`
`tip_3, tip_4`

### L30（17）

`j1, j2, j3, j4, j5, j6, j7, j8, j9, j10, j11, j12, j13, j14, j15, j16, j17`
