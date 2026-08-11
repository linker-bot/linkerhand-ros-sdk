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
| `publish_rate_hz` | `60.0` | ROS 读取缓存并发布状态的频率，不产生 I/O |
| `angles_hz` | `60.0` | SDK 角度查询频率，`0` 表示禁用 |
| `force_hz` | `0.0` | SDK 力传感器查询频率；当前无 ROS 力状态消息 |
| `temperature_hz` | `1.0` | SDK 温度查询频率，`0` 表示禁用 |
| `current_hz` | `10.0` | SDK 电流查询频率，`0` 表示禁用 |
| `faults_hz` | `5.0` | SDK 故障查询频率，`0` 表示禁用 |
| `diagnostic_rate` | `1.0` | `/diagnostics` 发布频率 |

请求的传感器频率会被限制到 SDK capability 的 `recommended_hz`；型号不支持的传感器
自动设为 `0`。`publish_rate_hz` 与设备查询频率相互独立，即使把发布频率设得更高也
不会增加总线查询次数。

节点在 configure 时一次性校验并保存全部参数。configured、active 或 inactive 状态下
修改参数不会改变当前连接；新值会在 cleanup 后再次 configure 时统一生效。

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
    force_hz: 0.0
    temperature_hz: 1.0
    current_hz: 10.0
    faults_hz: 5.0
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
