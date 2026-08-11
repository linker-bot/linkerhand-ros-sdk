# 使用示例

以下示例默认节点为 `/linkerbot_hand`，并假定 Lifecycle 状态为 active：

```bash
ros2 lifecycle get /linkerbot_hand
ros2 topic echo --once /linkerbot_hand/joint_names
ros2 topic hz /linkerbot_hand/state/angles
```

## L6 张开与握拳

所有型号统一为 `100=完全张开，0=完全握拳`。L6 顺序为：

```text
thumb_flex, thumb_abd, index, middle, ring, pinky
```

实机首次动作先设置低速度，并从中间值做小范围验证：

```bash
ros2 topic pub --once /linkerbot_hand/command/speeds \
  std_msgs/msg/Float64MultiArray "{data: [20, 20, 20, 20, 20, 20]}"

ros2 topic pub --once /linkerbot_hand/command/angles \
  std_msgs/msg/Float64MultiArray "{data: [60, 60, 60, 60, 60, 60]}"
```

确认安全后，完全张开和完全握拳分别为：

```text
[100, 100, 100, 100, 100, 100]
[0, 0, 0, 0, 0, 0]
```

`ros2 topic pub --once` 会为每条命令创建临时进程并等待 DDS 发现，不能用它的总耗时
衡量 ROS 回调、SDK dispatch 或 CAN 延迟，也不适合连续控制。

## 常驻交互控制器

```bash
ros2 run linkerbot_hand_ros angle_controller
```

控制器启动一次后复用同一个 Publisher：

```text
angles> 60 60 60 60 60 60
angles> 80 80 80 80 80 80
angles> 100 100 100 100 100 100
```

输入 `quit`、`exit` 或 `Ctrl-D` 退出。它是人工 bring-up 工具，每条输入会显示 DDS
确认时间；业务高频循环应直接使用 `HandCommandClient` 的默认非阻塞发送。

指定 namespace 内的驱动：

```bash
ros2 run linkerbot_hand_ros angle_controller \
  --driver-name /left_hand/linkerbot_hand
```

## 单次 Python 命令

```bash
ros2 run linkerbot_hand_ros angle_command \
  --values 100 100 100 100 100 100
```

该命令会等待驱动订阅者和 DDS 可靠传输确认，但不会等待 SDK `dispatched`、设备 ACK 或
机械到位。由于仍需启动新进程，它也不是低延迟连续控制接口。

## 业务循环

调用方创建一次客户端并持续复用：

```python
from linkerbot_hand_ros.client import HandCommandClient


class HandControlMixin:
    def initialize_hand(self):
        self.hand = HandCommandClient(self, "/linkerbot_hand")

    def update_hand_target(self, values):
        if not self.hand.is_ready("angles"):
            return None
        return self.hand.send_angles(values)
```

`send_angles()` 默认不等待 DDS ACK，驱动收到后只执行 SDK `submit_angles()` 并立即返回
callback；transport 由 SDK 唯一 worker 调度。

## 状态监听

```bash
ros2 run linkerbot_hand_ros state_monitor
```

监听 namespace 内的驱动：

```bash
ros2 run linkerbot_hand_ros state_monitor \
  --names-topic /left_hand/linkerbot_hand/joint_names \
  --angles-topic /left_hand/linkerbot_hand/state/angles
```

程序按 `joint_names` 输出百分比。某个轴没有独立百分比回读时显示 `nan`；业务代码必须
把它当作不可用于到位判断，而不是 0。

## L30 范围示例

L30 有 17 个关节，速度为 1–250 整数，扭矩为 60–800 整数。以下只是数组形状示例，
不要在未确认实物安全时直接执行端点命令：

```bash
ros2 topic pub --once /linkerbot_hand/command/speeds \
  std_msgs/msg/Float64MultiArray \
  "{data: [30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30]}"

ros2 topic pub --once /linkerbot_hand/command/torques \
  std_msgs/msg/Float64MultiArray \
  "{data: [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100]}"
```

L30 的百分比角度仍遵循全 100 张开、全 0 握拳，不再由 ROS 按协议最小/最大值自行
解释或反向。
