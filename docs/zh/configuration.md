# 节点配置

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model` | `l6` | `o6`、`l6`、`l20lite`、`l25`、`o20`、`o30i` 或 `l30` |
| `side` | `left` | `left` 或 `right`；L30/O30I 不使用该参数 |
| `interface_name` | `can0` | SocketCAN 接口名称 |
| `interface_type` | `socketcan` | `socketcan` 或 `ctypes` |
| `library_path` | 空 | `ctypes` 后端使用的厂商动态库路径 |
| `poll_rate` | `30.0` | 角度状态轮询频率，单位 Hz |
| `timeout_ms` | `100.0` | SDK 阻塞读取超时，单位 ms |
| `publish_optional_state` | `true` | 是否读取温度、电流和故障状态 |
| `optional_state_rate` | `1.0` | 可选状态轮询频率，单位 Hz |

`poll_rate`、`timeout_ms` 和 `optional_state_rate` 必须大于 0。总线负载较高或设备
较多时，应降低轮询频率。

## 使用 YAML

仓库中的 [`config/hand.yaml`](../../config/hand.yaml) 是 L6 示例。复制后修改参数：

```yaml
linkerbot_hand:
  ros__parameters:
    model: l6
    side: left
    interface_name: can0
    interface_type: socketcan
    library_path: ""
    poll_rate: 30.0
    timeout_ms: 100.0
    publish_optional_state: true
    optional_state_rate: 1.0
```

通过 `ros2 run` 加载：

```bash
ros2 run linkerbot_hand_ros hand_node \
  --ros-args --params-file /path/to/hand.yaml
```

## 使用命名空间

部署左右两只手时必须为节点设置不同命名空间，并按硬件拓扑选择 CAN 接口：

```bash
ros2 run linkerbot_hand_ros hand_node \
  --ros-args -r __ns:=/left_hand -p model:=l6 -p side:=left -p interface_name:=can0

ros2 run linkerbot_hand_ros hand_node \
  --ros-args -r __ns:=/right_hand -p model:=l6 -p side:=right -p interface_name:=can1
```

对应角度命令话题为：

- `/left_hand/linkerbot_hand/command/angles`
- `/right_hand/linkerbot_hand/command/angles`

## 后端选择

| 型号 | 总线 | 推荐后端 | 接口参数 |
| --- | --- | --- | --- |
| O6、L6、L20Lite、L25 | 经典 CAN | `socketcan` | `interface_name` |
| O20、O30I、L30 | CAN FD | `socketcan` | `interface_name` |
| O20、O30I、L30 | 厂商 CAN FD 适配器 | `ctypes` | `library_path` |

`ctypes` 后端下 `interface_name` 不参与厂商适配器选择；设备和通道使用 SDK 默认值。
