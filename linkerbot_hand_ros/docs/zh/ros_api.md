# ROS 2 接口

默认节点名为 `/linkerbot_hand`。源码使用 `~/` 私有名称，namespace 会自动加入所有
私有接口。状态发布器和运动命令订阅者只在 Lifecycle `active` 状态工作。

## 角度约定

所有屈伸轴对外统一为：

```text
100 = 完全张开
0   = 完全握拳
```

该值是百分比，不是弧度。侧摆、横滚、旋转和手腕等方向轴使用 capability 声明的
`zero_endpoint` 和 `hundred_endpoint`，0/100 是两个位置端点，不是握拳/张开。SDK
capability 是关节顺序、范围、整数约束、传感器能力和原始值映射的唯一数据源，ROS
不维护第二份型号表。

`raw_angles` 始终表示 SDK unified API 的协议原值（`protocol_raw`），即 CAN/CAN FD 实际
发送和读取的坐标。它不会经过 L6/O6/L20Lite/L25 旧 manager 的用户 LUT，也不等同于旧
`set_raw_angles()` 使用的 `standard_raw`。原始值范围和单位必须逐关节读取 capability。

协议角度样本由 SDK capability 映射后发布到 `state/angles`。无效、过期或无法完成映射的
整组样本不会发布。百分比不能直接写入 `sensor_msgs/JointState.position`；只有 capability
明确提供真实物理角度映射时，业务层才能转换成弧度。

## 话题

| 完整话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/linkerbot_hand/command/angles` | `std_msgs/msg/Float64MultiArray` | 订阅 | 张开度目标，0–100 |
| `/linkerbot_hand/command/raw_angles` | `std_msgs/msg/Float64MultiArray` | 订阅 | 精确协议原始角度目标，范围逐关节读取 capability |
| `/linkerbot_hand/command/speeds` | `std_msgs/msg/Float64MultiArray` | 订阅 | SDK capability 定义的速度目标 |
| `/linkerbot_hand/command/torques` | `std_msgs/msg/Float64MultiArray` | 订阅 | SDK capability 定义的扭矩目标 |
| `/linkerbot_hand/command/accepted/angles` | `std_msgs/msg/Float64MultiArray` | 发布 | SDK 已接受的归一化角度目标 |
| `/linkerbot_hand/command/accepted/raw_angles` | `std_msgs/msg/Float64MultiArray` | 发布 | SDK 已接受的协议原始角度目标 |
| `/linkerbot_hand/command/accepted/speeds` | `std_msgs/msg/Float64MultiArray` | 发布 | SDK 已接受的速度目标，单位来自 capability |
| `/linkerbot_hand/command/accepted/torques` | `std_msgs/msg/Float64MultiArray` | 发布 | SDK 已接受的电流/扭矩限制目标，单位来自 capability |
| `/linkerbot_hand/state/angles` | `std_msgs/msg/Float64MultiArray` | 发布 | 有效且未过期的缓存角度百分比 |
| `/linkerbot_hand/state/raw_angles` | `std_msgs/msg/Float64MultiArray` | 发布 | 同次有效角度样本包含的精确协议原值 |
| `/linkerbot_hand/state/speeds` | `std_msgs/msg/Float64MultiArray` | 发布 | 型号支持时发布有效速度缓存 |
| `/linkerbot_hand/state/torques` | `std_msgs/msg/Float64MultiArray` | 发布 | 型号支持时发布有效力矩缓存 |
| `/linkerbot_hand/state/temperature` | `std_msgs/msg/Float64MultiArray` | 发布 | 有效且未过期的缓存温度 |
| `/linkerbot_hand/state/current` | `std_msgs/msg/Float64MultiArray` | 发布 | 有效且未过期的缓存电流 |
| `/linkerbot_hand/state/fault` | `std_msgs/msg/Float64MultiArray` | 发布 | 有效且未过期的缓存故障值 |
| `/linkerbot_hand/state/voltage` | `std_msgs/msg/Float64MultiArray` | 发布 | 型号支持时发布有效电压缓存 |
| `/linkerbot_hand/state/acceleration` | `std_msgs/msg/Float64MultiArray` | 发布 | 型号支持时发布有效加速度缓存 |
| `/linkerbot_hand/state/motion_time` | `std_msgs/msg/Float64MultiArray` | 发布 | 型号支持时发布有效运动时间缓存 |
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

最近连续命令和离散管理操作的 ID、stage 和错误码位于 `/diagnostics`：

```text
latest_angles_command_id / latest_angles_stage / latest_angles_error_code
latest_raw_angles_command_id / latest_raw_angles_stage / latest_raw_angles_error_code
latest_speeds_command_id / latest_speeds_stage / latest_speeds_error_code
latest_torques_command_id / latest_torques_stage / latest_torques_error_code
latest_clear_faults_command_id / latest_clear_faults_stage / latest_clear_faults_error_code
latest_enable_command_id / latest_enable_stage / latest_enable_error_code
latest_disable_command_id / latest_disable_stage / latest_disable_error_code
latest_calibrate_command_id / latest_calibrate_stage / latest_calibrate_error_code
```

O30I 的连续命令支持设备协议 ACK。L6 等不支持 ACK 的型号以 `dispatched` 为成功终态。
机械到位必须由业务层使用同一 `connection_generation` 内有效且未过期的状态、目标
误差和稳定时间判断。

## 服务

| 服务 | 类型 | 语义 |
| --- | --- | --- |
| `/linkerbot_hand/read_now` | `std_srvs/srv/Trigger` | 立即发布当前新鲜角度缓存，不产生设备 I/O |
| `/linkerbot_hand/clear_faults` | `std_srvs/srv/Trigger` | 提交异步清错命令，成功仅表示 SDK 接受 |
| `/linkerbot_hand/enable` | `std_srvs/srv/Trigger` | 型号支持时提交异步使能命令 |
| `/linkerbot_hand/disable` | `std_srvs/srv/Trigger` | 型号支持时提交异步禁用命令 |
| `/linkerbot_hand/calibrate` | `std_srvs/srv/Trigger` | 型号支持时提交异步标定命令 |

```bash
ros2 service call /linkerbot_hand/read_now std_srvs/srv/Trigger '{}'
ros2 service call /linkerbot_hand/clear_faults std_srvs/srv/Trigger '{}'
ros2 service call /linkerbot_hand/enable std_srvs/srv/Trigger '{}'
ros2 service call /linkerbot_hand/disable std_srvs/srv/Trigger '{}'
ros2 service call /linkerbot_hand/calibrate std_srvs/srv/Trigger '{}'
```

当前 SDK capability 中 L6、L25 和 O20 支持清错；其他型号会明确返回
`success=false`。L30 支持 enable、disable 和 calibrate；其他型号会按 capability 拒绝。
Trigger 的 `success=true` 只表示 SDK 接受，dispatch 或协议 ACK 应从诊断读取。inactive
状态下所有管理服务都不会操作设备。

触觉 force 通过 `/linkerbot_hand/state/force` 发布行优先三维
`Float64MultiArray`，布局维度为 `finger/row/column`，手指顺序为
`thumb/index/middle/ring/pinky`。O6 `04-B` 的布局为 `5x10x4`，L6 为
`5x12x6`。兼容消息仅携带数值和形状；采集时间、序号、有效性、过期状态和 connection
generation 仍通过 SDK snapshot 与 ROS diagnostics 获取。

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
`superseded_commands`、各传感器 `*_age_s`、`*_sequence`、`*_valid`、
`*_stale`、`*_collected_at`、`*_connection_generation`、SDK 稳定错误码和最近命令阶段。
`ros_accepted_feedback_errors` 单独统计 SDK 已接受命令之后的 ROS 反馈发布失败，
不会把可能已经执行的命令误记为 rejected。

`state/*` 可能按 ROS 发布定时器重复发布同一缓存。测量真实设备 polling 频率必须使用同一
`*_connection_generation` 内的 `*_sequence` 增量，不能只读取 `ros2 topic hz`。完整测试
流程见[关节命令与状态频率测试](frequency_benchmark.md)。

## 命令范围

以下为当前 SDK 0.2.x capability 摘要，运行时仍以实际 capability 为准：

| 型号 | 百分比角度 | 协议速度 | 协议扭矩/电流限制 |
| --- | --- | --- | --- |
| L6、O6、L20Lite、L25 | 0–100 | 0–255 整数 count | 0–255 整数 count |
| O20 | 0–100 | 0–100 整数 count | 0–1000 整数 count |
| O30I | 0–100 | 0–255 整数 | 0–255 整数 |
| L30 | 0–100 | 1–250 整数 count | 60–800 整数 count |

ROS 话题类型为 Float64 数组；驱动会按 SDK capability 校验并把要求整数的 Float64 转成
整数，带小数的整数型命令会被拒绝。除百分比角度外，ROS 数值命令和状态话题直接暴露
SDK `SensorSample.values`，单位读取 `SensorCapability.unit`。SDK 0.2.x 默认使用协议解析
值，常见为 `count`；`physical_unit/scale/offset` 是物理解释元数据，ROS 不隐式换算。
O20 当前电流已经由协议解析为 mA。L6 能设置速度，但没有当前速度查询；因此目标会出现在
`command/accepted/speeds`，不会发布伪造的 `/state/speeds`。

## API 迁移顺序

这里的顺序是实现替换顺序，不是包发布顺序：

1. ROS 只依赖独立 `linkerbot-hand`，型号与范围只读取 `get_capabilities()`。
2. Lifecycle activate/deactivate 显式启动和停止 SDK `PollingPlan`；ROS 状态定时器只读取缓存。
3. 百分比 topic 调用 `submit_percentage_angles()`，协议 raw topic 调用
   `submit_angles()`，两套坐标不混用。
4. 所有运动 callback 只保存非阻塞 `CommandHandle`，诊断按准确阶段和稳定错误码转换。
5. 恢复由 SDK 管理且不重放旧目标，最后使用 fake transport 和逐轴实物测试验证。

`linkerbot-hand` 拥有能力、角度换算、轮询、调度和缓存；本 ROS 仓库只拥有参数、
Lifecycle、topic/service、消息转换和诊断，不重复实现型号映射。

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
