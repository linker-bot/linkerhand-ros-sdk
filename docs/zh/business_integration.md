# 业务节点集成

## 运行架构

正式部署中，每只手只运行一个驱动节点。驱动节点独占 Python SDK 和 CAN 连接，
业务节点通过长期存在的 `HandCommandClient` 与驱动通信：

```text
业务节点（HandCommandClient，常驻 Publisher）
    |  command/*                  ^  command/accepted/*、joint_names、model
    v                             |
linkerbot_hand（唯一 SDK/CAN 所有者）
    |  Python SDK
    v
灵巧手硬件
```

不要让多个进程直接实例化同一只手的 Python SDK，也不要在控制循环中反复创建 ROS
节点或 Publisher。允许多个状态监听者，但同一只手应由一个业务控制者负责发命令。

## Python API

`HandCommandClient` 使用调用方已有的 ROS 2 Node，不创建线程、执行器或硬件连接：

```python
import rclpy
from rclpy.node import Node

from linkerbot_hand_ros.client import HandCommandClient


class ProductionNode(Node):
    def __init__(self):
        super().__init__("hand_business")
        self.hand = HandCommandClient(self, "/linkerbot_hand")

    def set_hand_target(self, values):
        if not self.hand.is_ready("angles"):
            raise RuntimeError("hand driver is not ready")
        return self.hand.send_angles(values)

    def destroy_node(self):
        self.hand.destroy()
        return super().destroy_node()


def main():
    rclpy.init()
    node = ProductionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

`send_angles`、`send_speeds` 和 `send_torques` 默认只执行非阻塞发布，适合业务控制
循环。离散操作需要确认 DDS 可靠传输时，可显式使用：

```python
receipt = hand.send_angles(
    [100, 50, 100, 100, 100, 100],
    wait_for_dds_ack=True,
    timeout_sec=1.0,
)
```

不要在高频循环中为每一帧等待 ACK。驱动的命令 QoS 深度为 1，控制循环应持续发布
最新目标；过期目标不会在队列中累积。

## 三层确认

| 层级 | 接口 | 含义 |
| --- | --- | --- |
| DDS 传输 | `CommandReceipt.dds_acknowledged` | ROS 2 订阅端可靠接收 |
| SDK 接受 | `command/accepted/*`、`last_accepted()` | 参数有效且 SDK setter 正常返回 |
| 机械到位 | `state/angles` | 实际角度进入业务定义的允许误差 |

业务流程必须按需要选择确认层级。例如抓取动作不能只依赖 DDS ACK，应等待实际角度
进入容差，并结合电流、故障和超时策略决定成功或失败。

## 诊断与监控

驱动按 `diagnostic_rate` 向 `/diagnostics` 发布标准状态，包含：

- 状态读取和命令成功/失败计数
- 最近状态与命令距当前时间
- 最近一次操作是否失败
- 最近错误文本、型号、关节数和运行时间

部署系统应监控诊断级别。当状态长期未更新时驱动报告 `ERROR`；最近一次读取或命令
失败时报告 `WARN`。诊断不能替代独立急停、机械限位、电流保护和上层动作超时。

## 生命周期

业务节点关闭时调用 `HandCommandClient.destroy()`；该操作可重复调用。驱动关闭时
会释放 Python SDK/CAN 资源。设备断线自动重连和 ROS 2 Lifecycle 管理尚未包含在
0.2.0，应由后续独立版本实现，当前部署可由 systemd 或容器编排负责进程重启。
