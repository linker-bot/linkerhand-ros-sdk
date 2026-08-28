# 从 0.3.x 兼容接口迁移到 Hand SDK 1.0 契约

本次迁移是有意的破坏性变更。新驱动不保留旧数组话题、旧 adapter 或旧参数别名，目标是
只提供一套清晰、可验证的接口。旧程序需要一次性迁移，不应同时运行两套驱动。

## 依赖

```bash
python3 -m pip install --upgrade \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-hand>=1.0.0.dev0,<2.0.0'
```

ROS 不依赖旧聚合 `linkerbot` 或 `linkerbot-python-sdk`。开发版可从独立
`linkerbot-transport` 和 `linkerbot-hand` 仓库 editable install。

## 参数迁移

| 旧参数 | 新参数/处理 |
| --- | --- |
| `transport_options_json` | `transport_json`，必须带明确 `backend` |
| `interface_name/interface_type/library_path` | 删除，进入 backend-specific JSON |
| `diagnostic_rate` | `diagnostic_rate_hz` |
| `acceleration_hz` | 删除；configured acceleration 不是周期测量通道 |
| `motion_time_hz` | 删除；motion time 不是周期测量通道 |
| 隐式 session 配置 | `DeviceConfig` 对应的队列、timeout 和重连参数全部显式公开 |

新 SocketCAN：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left \
  transport_json:='{"backend":"socketcan","channel":"can0"}'
```

## Topic 迁移

| 旧接口 | 新接口 |
| --- | --- |
| `command/angles` 数组 | `HandJointCommand`，`representation=1` percent |
| `command/raw_angles` | 同一个 `HandJointCommand`，`representation=2` raw |
| `command/speeds` 数组 | `HandRawCommand` |
| 含义混杂的 `command/torques` | capability 区分 current limits、effort limits、true torques |
| `state/angles` 百分比数组 + `state/raw_angles` | 一个 `HandJointState` 同时保存 raw/percent/physical |
| `state/speeds` | `measured_speeds` 或 `configured_speeds`，不可混用 |
| 其他通用数组状态 | `HandTelemetry`，显式 semantic、channels、raw 和 metadata |
| force 三维数组 | `HandTactileState`，每指矩阵和完整事务 metadata |
| `joint_names`、`model` 字符串 | `HandCapabilities` 和 `HandDeviceInfo` |
| `command/accepted/*` | 统一 `HandCommandStatus` 阶段流 |

新状态都携带 sequence、采集时间、age、valid、stale、connection generation 和 origin。
业务不再通过 diagnostics 拼接状态元数据。

## Service 迁移

- 删除 `read_now`。缓存状态由 state publisher 在新 sequence 时自动发布。
- `clear_faults/enable/disable/calibrate` 改用 `linkerbot_hand_msgs/srv/SubmitCommand`。
- 新增 `get_device_info`、`recover` 和堵转 raw 配置服务。
- 服务 setter 成功只表示 SDK `ACCEPTED`，后续阶段订阅 `/command/status`。

## 角度和值语义

- 删除 LUT、standard raw、user raw 和 hardware raw 多层命名。
- percent 明确为 `0..100`；屈伸轴 0 弯曲、100 伸直。
- 普通 raw 接口就是设备协议原值。
- 读取角度在同一消息中提供 raw、percent 和可选 physical radians。
- 速度、电流、限制、扭矩、温度、力和堵转配置不做隐式物理换算。

## 行为变化

- configure 零 I/O，activate 才 open 和身份验证。
- ROS command callback 不同步等待 SDK 或设备。
- SDK 是唯一 polling 所有者；ROS 只发布缓存。
- 同一 sequence 不重复发布。
- 连续命令 latest-wins，离散操作有界 FIFO。
- unsupported ACK 不再伪装成 acknowledged。
- recovery 清除旧目标并重新验证设备，禁止自动重放。

## 上线检查

1. 删除业务中的旧 topic 名、旧参数别名和本地型号表。
2. 从 capability 构造关节布局与命令支持矩阵。
3. 用 `user_sequence` 和 `HandCommandStatus` 管理送达状态。
4. 用当前 generation 的有效 `HandJointState` 判断到位。
5. 分别完成 percent/raw 小范围实机验证。
6. 停止 polling 后抓总线确认零查询。
7. 完成生命周期、断线恢复和频率 benchmark 后再替换旧部署。
