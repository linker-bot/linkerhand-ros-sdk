# Lifecycle 与参数配置

## Lifecycle 行为

| 状态转换 | 行为 |
| --- | --- |
| configure | 解析参数，构造 transport 描述、唯一 `HandDevice` 和 ROS 实体；零连接、零设备 I/O |
| activate | `open()`，验证型号/固件/传感器变体，安装 polling plan，启用命令和状态发布 |
| deactivate | 先撤销命令入口，再停止 SDK polling；不自动运动、不关闭连接 |
| cleanup | 有界停止 admin worker、polling 和设备 runtime，销毁 ROS 实体 |
| shutdown/error | 无论来自哪个状态都尝试有界 stop/close，不依赖析构函数 |

launch 默认 `autostart:=true`。外部 lifecycle manager 接管时：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py autostart:=false
ros2 lifecycle set /linkerbot_hand configure
ros2 lifecycle set /linkerbot_hand activate
```

## 静态参数

静态参数在 configure 后拒绝修改，需先 `deactivate -> cleanup`：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `model` | `l6` | `l6/o6/l20lite/l25/o20/l30/o30i` |
| `side` | `left` | `left/right`；L30 必须为空字符串 |
| `transport_json` | SocketCAN `can0` | 严格 JSON transport 配置 |
| `command_rate_hz` | `250` | SDK 连续命令调度上限 |
| `discrete_queue_capacity` | `16` | 离散和安全配置命令的有界 FIFO 容量 |
| `mailbox_capacity` | `256` | 设备 runtime mailbox 容量 |
| `route_queue_capacity` | `256` | transport 路由队列容量 |
| `identity_timeout_s` | `0.2` | 身份/版本事务超时 |
| `transaction_timeout_s` | `0.1` | 普通协议事务默认超时 |
| `shutdown_timeout_s` | `2.0` | worker 和连接关闭总边界 |
| `auto_reconnect` | `false` | SDK 自动安全恢复 |
| `reconnect_initial_delay_s` | `0.1` | 首次重连退避 |
| `reconnect_max_delay_s` | `2.0` | 最大重连退避 |

## 动态频率

| 参数 | 默认值 | 查询/发布对象 |
| --- | --- | --- |
| `angles_hz` | `100` | SDK 角度查询 |
| `speeds_hz` | `0` | SDK 实测速度查询 |
| `force_hz` | `0` | SDK force 或完整触觉事务 |
| `temperature_hz` | `0` | SDK 温度查询 |
| `current_hz` | `0` | SDK 实测电流查询 |
| `faults_hz` | `0` | SDK 故障查询 |
| `torques_hz` | `0` | SDK 真实扭矩查询，仅 capability 支持时 |
| `voltage_hz` | `0` | SDK 电压查询 |
| `publish_rate_hz` | `100` | ROS 检查缓存的频率 |
| `diagnostic_rate_hz` | `1` | `/diagnostics` 发布频率 |

非零 polling 只允许 capability 声明 `device_queried=true` 的通道。请求不支持通道时参数
事务失败，不会静默忽略。频率设为 0 后 SDK 不再生成该通道的查询；
`publish_rate_hz=0` 只暂停 ROS 状态发布，并不停止设备 polling。

```bash
ros2 param set /linkerbot_hand angles_hz 100.0
ros2 param set /linkerbot_hand force_hz 30.0
ros2 param set /linkerbot_hand publish_rate_hz 100.0
ros2 param set /linkerbot_hand diagnostic_rate_hz 10.0
```

运行中多个 rate 参数作为一次原子事务修改：

```bash
ros2 param set --help
```

需要严格原子更新时应通过 `rcl_interfaces/srv/SetParametersAtomically` 或代码中的
`AsyncParameterClient.set_parameters_atomically()`，频率 benchmark 已按此方式实现。

## Transport JSON

Linux SocketCAN，classic CAN 和 CAN FD 均由 capability 决定：

```json
{"backend":"socketcan","channel":"can0"}
```

可选字段：`nominal_bitrate`、`data_bitrate`、`auto_reconfigure`、
`receive_own_messages`。配置 bitrate 与 capability 冲突时 configure 失败。

其他 python-can classic backend：

```json
{"backend":"python_can","interface_name":"PCAN_USBBUS1","interface_type":"pcan"}
```

厂商 CAN FD：

```json
{"backend":"vendor_canfd","device_index":0,"channel_index":0,"library_path":"/path/to/lib.so"}
```

未知 backend、缺失必填键或多余键均为配置错误。旧
`transport_options_json/interface_name/interface_type/library_path` 参数已删除。

## 多设备

每只手使用独立 namespace：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  namespace:=left model:=l6 side:=left \
  transport_json:='{"backend":"socketcan","channel":"can0"}'
ros2 launch linkerbot_hand_ros hand.launch.py \
  namespace:=right model:=o6 side:=right \
  transport_json:='{"backend":"socketcan","channel":"can0"}'
```

同一物理总线上的设备应位于同一进程/component container，才能共享 transport runtime、
RX reactor 和 TX arbiter。两个独立进程只能各自拥有 socket，无法获得进程内公平调度保证。

## 恢复

自动或显式恢复会重新验证身份和固件、清除旧命令并读取 baseline 状态。不会重放断线前或
断线期间目标，不会自动张开、握拳、清错、enable 或 calibrate。业务层看到
`connection_generation` 增加后必须清空旧到位判断，并显式提交新目标。
