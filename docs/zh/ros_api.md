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
| `/linkerbot_hand/state/angles` | `std_msgs/msg/Float64MultiArray` | 发布 | 当前角度，0–100 |
| `/linkerbot_hand/state/temperature` | `std_msgs/msg/Float64MultiArray` | 发布 | 温度，设备支持时发布 |
| `/linkerbot_hand/state/current` | `std_msgs/msg/Float64MultiArray` | 发布 | 电流，设备支持时发布 |
| `/linkerbot_hand/state/fault` | `std_msgs/msg/Float64MultiArray` | 发布 | 原始故障值，设备支持时发布 |
| `/linkerbot_hand/joint_names` | `std_msgs/msg/String` | 发布 | 逗号分隔的关节顺序 |

所有数组必须严格按照 `joint_names` 的顺序和数量发送。命令长度或范围错误时，节点
会拒绝命令并记录错误，不会向硬件转发。

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
