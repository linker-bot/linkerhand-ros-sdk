# Lifecycle 与节点配置

## 生命周期

| ROS 状态转换 | SDK 行为 | 设备 I/O |
| --- | --- | --- |
| `on_configure` | 创建 `HandDevice`、读取静态能力 | 无连接、无查询 |
| `on_activate` | `open()`、`start_polling(PollingPlan)` | 打开唯一连接并开始轮询 |
| `on_deactivate` | 禁止命令、`stop_polling()` | 停止新查询，连接保留 |
| `on_cleanup` / `on_shutdown` | `close()` | 有界关闭连接和 worker |

inactive 状态没有运动命令订阅者，`HandCommandClient.is_ready()` 会返回 `false`。
deactivate 不会自动张开、握拳、清错或重放目标。

## 节点参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model` | `l6` | `o6/l6/l20lite/l25/o20/o30i/l30` |
| `side` | `left` | `left/right/unknown`；具体型号要求来自 SDK |
| `transport_options_json` | `{}` | 原样传给 SDK 工厂的 JSON 对象，非空时优先 |
| `command_rate_hz` | `100.0` | SDK 连续命令最大 transport 发送频率 |
| `auto_reconnect` | `false` | 是否使用 SDK 指数退避自动恢复 |
| `publish_rate_hz` | `60.0` | ROS 读取缓存并发布状态的频率；`0` 暂停全部状态话题 |
| `angles_hz` | `60.0` | SDK 角度查询频率，`0` 表示禁用 |
| `speeds_hz` | `0.0` | SDK 速度查询频率，`0` 表示禁用 |
| `torques_hz` | `0.0` | SDK 力矩查询频率，`0` 表示禁用 |
| `force_hz` | `0.0` | SDK 力传感器查询频率；当前无 ROS 力状态消息 |
| `temperature_hz` | `1.0` | SDK 温度查询频率，`0` 表示禁用 |
| `current_hz` | `10.0` | SDK 电流查询频率，`0` 表示禁用 |
| `faults_hz` | `5.0` | SDK 故障查询频率，`0` 表示禁用 |
| `voltage_hz` | `0.0` | SDK 电压查询频率，`0` 表示禁用 |
| `acceleration_hz` | `0.0` | SDK 加速度查询频率，`0` 表示禁用 |
| `motion_time_hz` | `0.0` | SDK 运动时间查询频率，`0` 表示禁用 |
| `diagnostic_rate` | `1.0` | `/diagnostics` 发布频率；`0` 表示暂停 |

型号支持的传感器频率会按用户请求值原样交给 SDK；capability 的 `recommended_hz`
仅是推荐值，不是硬上限。型号不支持的传感器自动设为 `0`。`publish_rate_hz` 与设备
查询频率相互独立，即使把发布频率设得更高也不会增加总线查询次数。

上述轮询频率、`publish_rate_hz` 和 `diagnostic_rate` 可在 launch 后动态修改。active
状态下传感器频率会原子替换 SDK `PollingPlan`，不会重建 CAN 连接；设为 `0` 会停止
对应设备查询和状态发布。configured/inactive 状态下修改会在下次 activate 生效。

```bash
# L6 在线启用 30 Hz 力矩反馈，再关闭
ros2 param set /linkerbot_hand torques_hz 30.0
ros2 topic echo /linkerbot_hand/state/torques
ros2 param set /linkerbot_hand torques_hz 0.0

# 暂停并恢复全部缓存状态发布；不会改变 SDK 设备查询频率
ros2 param set /linkerbot_hand publish_rate_hz 0.0
ros2 param set /linkerbot_hand publish_rate_hz 60.0

# 暂停并恢复诊断
ros2 param set /linkerbot_hand diagnostic_rate 0.0
ros2 param set /linkerbot_hand diagnostic_rate 1.0
```

当前 SDK capability 的速度、力矩回读能力如下。型号不支持的频率会被限制为 `0`，
不会发起无效设备查询；型号能力以安装的 SDK 版本为准。

| 型号 | `/state/speeds` | `/state/torques` |
| --- | --- | --- |
| L6 | 不支持 | 支持 |
| O6 | 支持 | 支持 |
| L20Lite | 支持 | 支持 |
| L25 | 支持 | 支持 |
| O20 | 支持 | 不支持 |
| O30I | 支持 | 支持 |
| L30 | 支持 | 不支持 |

其他参数在 configure 时保存快照。`model`、`side`、transport、`command_rate_hz` 和
`auto_reconnect` 在 configured/inactive/active 状态会被明确拒绝；必须先 deactivate、
cleanup，修改后再 configure/activate。unconfigured 状态设置参数时会立即使用 SDK
能力和完整候选配置校验型号、side、JSON 和频率，不会把无效值暂存到参数服务器。

O30I 支持 `voltage/acceleration/motion_time`，O6 支持 `acceleration`；其他型号若 capability
未声明，对应非零频率会被限制为 `0`，不会发送无效查询。`force_hz` 可以启动 SDK 触觉
矩阵轮询，但当前 ROS 兼容接口不会将二维触觉矩阵有损压成 `Float64MultiArray`，需等待
专用强类型消息。

launch 另外提供 `namespace` 和 `autostart` 参数，它们不是节点参数。

## Transport JSON

推荐显式使用 JSON，避免 ROS 按型号硬编码构造函数：

| 型号 | SocketCAN 示例 |
| --- | --- |
| L6/O6/L20Lite/L25 | `{"interface_name":"can0","interface_type":"socketcan"}` |
| L30 | `{"interface_type":"socketcan","channel":"can0","node_id":1}` |
| O20 | `{"interface_type":"socketcan","socketcan_channel":"can0"}` |
| O30I | `{"interface_type":"socketcan","socketcan_channel":"can0","request_id":1}` |

`model`、`side`、`session_config` 和 `clock` 由 ROS 节点管理，不能写入 JSON。厂商动态
库模式的键名和取值以对应 SDK 型号文档为准。

为兼容旧启动命令，节点仍声明 `interface_name`、`interface_type` 和 `library_path`。
它们仅在 `transport_options_json` 为空对象时生效，新部署不要混用两套配置。

## YAML

仓库的 [`config/hand.yaml`](../../config/hand.yaml) 是 L6 示例：

```yaml
linkerbot_hand:
  ros__parameters:
    model: l6
    side: left
    transport_options_json: '{"interface_name":"can0","interface_type":"socketcan"}'
    command_rate_hz: 100.0
    auto_reconnect: false
    publish_rate_hz: 60.0
    angles_hz: 60.0
    speeds_hz: 0.0
    torques_hz: 0.0
    force_hz: 0.0
    temperature_hz: 1.0
    current_hz: 10.0
    faults_hz: 5.0
    voltage_hz: 0.0
    acceleration_hz: 0.0
    motion_time_hz: 0.0
    diagnostic_rate: 1.0
```

直接运行节点时不会自动切换 Lifecycle 状态：

```bash
ros2 run linkerbot_hand_ros hand_node \
  --ros-args --params-file /path/to/hand.yaml
ros2 lifecycle set /linkerbot_hand configure
ros2 lifecycle set /linkerbot_hand activate
```

## 左右手部署

每只手必须有独立 namespace、CAN 拓扑和唯一驱动节点：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  namespace:=/left_hand model:=l6 side:=left \
  transport_options_json:='{"interface_name":"can0","interface_type":"socketcan"}'

ros2 launch linkerbot_hand_ros hand.launch.py \
  namespace:=/right_hand model:=l6 side:=right \
  transport_options_json:='{"interface_name":"can1","interface_type":"socketcan"}'
```

对应节点分别为 `/left_hand/linkerbot_hand` 和 `/right_hand/linkerbot_hand`。

## 自动重连

`auto_reconnect:=true` 时，SDK 在唯一 worker 内执行指数退避恢复。恢复会增加
`connection_generation`、清空旧运动目标，并等待新一代状态；ROS 不会重放断线前或
断线期间的命令。业务节点必须观察诊断恢复正常后显式下发新目标。
