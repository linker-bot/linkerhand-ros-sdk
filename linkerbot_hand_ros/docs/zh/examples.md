# 命令行与 Python 示例

示例默认驱动为 `/linkerbot_hand` 且 Lifecycle 为 active。先读取 capability，不要硬编码
型号关节表：

```bash
ros2 topic echo --once /linkerbot_hand/capabilities
ros2 topic echo --once /linkerbot_hand/state/angles
```

## Percent 角度

```bash
ros2 topic pub --once /linkerbot_hand/command/angles \
  linkerbot_hand_msgs/msg/HandJointCommand \
  '{client_id: 1, user_sequence: 1, representation: 1, values: [50, 50, 50, 50, 50, 50]}'
```

屈伸轴 `0=弯曲，100=伸直`。侧摆和旋转轴查看 capability 的 zero/hundred label。

## 协议 raw 角度

```bash
ros2 topic pub --once /linkerbot_hand/command/angles \
  linkerbot_hand_msgs/msg/HandJointCommand \
  '{client_id: 1, user_sequence: 2, representation: 2, values: [255, 128, 204, 178, 153, 0]}'
```

raw 不经过 percent、方向、LUT 或物理单位转换。值必须满足逐关节 capability 范围。

## 速度与限制值

仅在 capability 声明支持时发布对应 Topic：

```bash
ros2 topic pub --once /linkerbot_hand/command/speeds \
  linkerbot_hand_msgs/msg/HandRawCommand \
  '{client_id: 1, user_sequence: 3, raw: [20, 20, 20, 20, 20, 20]}'
```

电流上限、effort 上限和真实扭矩是不同接口：

```bash
ros2 topic pub --once /linkerbot_hand/command/current_limits \
  linkerbot_hand_msgs/msg/HandRawCommand \
  '{client_id: 1, user_sequence: 4, raw: [80, 80, 80, 80, 80, 80]}'
```

不要因为旧协议表曾写“扭矩”而改发 `/command/torques`。以 capability 的 command
semantic 为准。

## 命令状态

```bash
ros2 topic echo /linkerbot_hand/command/status
```

用 `(client_id, user_sequence)` 关联目标。无 ACK 型号在 `DISPATCHED` 成功终止；有协议 ACK 的型号等待
`ACKNOWLEDGED`。Topic 类型是 `HandCommandStatusBatch`，直接订阅时逐条处理 `statuses[]`；
机械到位由 `state/angles` 的新鲜反馈判断。

## 常驻 Python 客户端

```python
import rclpy
from rclpy.node import Node

from linkerbot_hand_ros.client import HandCommandClient


class Controller(Node):
    def __init__(self) -> None:
        super().__init__("hand_controller")
        self.hand = HandCommandClient(self, "/linkerbot_hand")
        self.timer = self.create_timer(0.01, self.tick)

    def tick(self) -> None:
        if not self.hand.is_ready("angles"):
            return
        publication = self.hand.set_percent_angles([50.0] * len(self.hand.joint_names))
        self.get_logger().debug(f"published user_sequence={publication.user_sequence}")

    def destroy_node(self) -> None:
        self.hand.destroy()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

`set_percent_angles()` 表示发布成功，不表示 SDK 已接受。SDK stage 从 status topic 到达。
不要在定时 callback 中调用 `wait_for_terminal_status()`；需要等待时使用另一线程或异步状态机，
并保证 executor 仍在处理 status。

仓库内置：

```bash
ros2 run linkerbot_hand_ros angle_command --values 50 50 50 50 50 50
ros2 run linkerbot_hand_ros angle_controller
ros2 run linkerbot_hand_ros state_monitor
```

## 主动设备信息和恢复

```bash
ros2 service call /linkerbot_hand/get_device_info \
  linkerbot_hand_msgs/srv/GetDeviceInfo '{timeout_s: 0.2}'
ros2 service call /linkerbot_hand/recover \
  linkerbot_hand_msgs/srv/Recover '{timeout_s: 1.0}'
```

恢复不会重放旧目标。成功后等待新 `connection_generation` 的有效角度，再显式发送目标。

## 堵转配置

先读取，不猜单位或安全范围：

```bash
ros2 service call /linkerbot_hand/get_stall_protection \
  linkerbot_hand_msgs/srv/GetStallProtection '{timeout_s: 0.5}'
```

未确认安全范围前，实机只允许读取当前值并原样写回验证通信：

```bash
ros2 service call /linkerbot_hand/set_stall_thresholds \
  linkerbot_hand_msgs/srv/SetRawValues \
  '{user_sequence: 10, raw: [<按 capability 通道顺序填写>]}'
```

如果 capability 的 `operations.save_parameters=true`，需要持久化时再调用：

```bash
ros2 service call /linkerbot_hand/save_parameters \
  linkerbot_hand_msgs/srv/SubmitCommand '{}'
```

两个 service 的成功响应都只表示 SDK 接受；O6/L6 的配置写和保存操作应继续观察
`/command/status`，以各自 `command_id` 确认最终 `ACKNOWLEDGED`。不要在未确认当前值、安全
范围和恢复方法时写任意堵转参数，也不要把 O6 的 `C5/C6` 顺序套用到 L6。

## 完整触觉

```bash
ros2 topic echo /linkerbot_hand/state/tactile
ros2 topic hz /linkerbot_hand/state/tactile
```

每条 `HandTactileState` 是一次完整多指事务。频率按整个消息计数，不把矩阵 cell、CAN 分片
或单个手指当作独立样本。
