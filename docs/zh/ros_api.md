# ROS 2 接口

节点默认名为 `/linkerbot_hand`。源码中使用 `~/` 私有名称，因此默认话题
`~/state/angles` 的完整名称是 `/linkerbot_hand/state/angles`。设置命名空间后，
完整名称会随命名空间变化。

## 话题

| 完整话题（默认节点名） | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/linkerbot_hand/command/angles` | `std_msgs/msg/Float64MultiArray` | 订阅 | 角度目标，0–100 |
| `/linkerbot_hand/command/speeds` | `std_msgs/msg/Float64MultiArray` | 订阅 | 各关节速度目标 |
| `/linkerbot_hand/command/torques` | `std_msgs/msg/Float64MultiArray` | 订阅 | 各关节扭矩目标 |
| `/linkerbot_hand/command/accepted/angles` | `std_msgs/msg/Float64MultiArray` | 发布 | SDK 已接受的角度命令 |
| `/linkerbot_hand/command/accepted/speeds` | `std_msgs/msg/Float64MultiArray` | 发布 | SDK 已接受的速度命令 |
| `/linkerbot_hand/command/accepted/torques` | `std_msgs/msg/Float64MultiArray` | 发布 | SDK 已接受的扭矩命令 |
| `/linkerbot_hand/state/angles` | `std_msgs/msg/Float64MultiArray` | 发布 | 当前角度，0–100 |
| `/linkerbot_hand/state/temperature` | `std_msgs/msg/Float64MultiArray` | 发布 | 温度，设备支持时发布 |
| `/linkerbot_hand/state/current` | `std_msgs/msg/Float64MultiArray` | 发布 | 电流，设备支持时发布 |
| `/linkerbot_hand/state/fault` | `std_msgs/msg/Float64MultiArray` | 发布 | 原始故障值，设备支持时发布 |
| `/linkerbot_hand/joint_names` | `std_msgs/msg/String` | 发布 | 逗号分隔的关节顺序 |
| `/linkerbot_hand/model` | `std_msgs/msg/String` | 发布 | 规范化型号名称 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 发布 | 状态新鲜度、计数和最近错误 |

所有数组必须严格按照 `joint_names` 的顺序和数量发送。命令长度或范围错误时，节点
会拒绝命令并记录错误，不会向硬件转发。

命令订阅使用 `RELIABLE + KEEP_LAST(depth=1) + VOLATILE`，连续控制时只保留最新
尚未处理的目标，避免硬件追赶已经过期的命令。模型、关节顺序和 accepted 反馈使用
`RELIABLE + KEEP_LAST(depth=1) + TRANSIENT_LOCAL`，后启动的业务客户端可立即获得
最近元数据与反馈。

`command/accepted/*` 表示参数校验通过且对应 Python SDK `set_*` 调用已正常返回，
不表示机械关节已经到达目标。实际到位必须结合 `state/angles` 和业务容差判断；DDS
ACK 只表示 ROS 2 中间件完成可靠传输，三者不能混用。

角度值 `0–100` 只表示 SDK 的逻辑范围，不是跨型号统一的张开/闭合定义：

- L6 屈伸关节：`0` 向握拳方向，`100` 向张开方向；拇指侧摆需单独看待。
- O30I：SDK 明确定义为 `0=open`、`100=closed`，与 L6 相反。
- O20/L30：`0/100` 分别映射到各关节协议最小/最大值，物理方向取决于关节。
- 其余型号：按对应 SDK 和硬件文档确认，不应从数值大小推断物理方向。

## 服务

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `/linkerbot_hand/read_now` | `std_srvs/srv/Trigger` | 立即读取并发布一次角度 |
| `/linkerbot_hand/clear_faults` | `std_srvs/srv/Trigger` | 型号支持时发送故障清除命令 |

```bash
ros2 service call /linkerbot_hand/read_now std_srvs/srv/Trigger '{}'
ros2 service call /linkerbot_hand/clear_faults std_srvs/srv/Trigger '{}'
```

O30I 和 L30 当前 SDK 只开放故障读取，不开放故障清除；调用清除服务会返回
`success=false` 及原因，节点不会退出。

## 命令范围

| 型号 | 角度 | 速度 | 扭矩 |
| --- | --- | --- | --- |
| O6、L6、L20Lite、L25 | 0–100 | 0–100 | 0–100 |
| O20 | 0–100 | 0–100 整数 | 0–1000 整数 |
| O30I | 0–100 | 0–255 整数 | 0–255 整数 |
| L30 | 0–100 | 1–250 整数 | 60–800 整数 |

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
