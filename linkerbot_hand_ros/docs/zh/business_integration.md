# 正式业务节点集成

## 所有权边界

每只实体手只允许一个 ROS 驱动节点和一个 `HandDevice`：

```text
业务控制节点（常驻）
    | HandCommandClient / command/*
    v
Lifecycle 驱动（唯一 SDK 所有者）
    | submit_* / get_snapshot / get_health
    v
SDK scheduler + 唯一 I/O worker
    v
transport / protocol / hand
```

业务节点不能同时直接实例化同一只手的 SDK，ROS executor 不能增加第二个传感器查询
线程，也不能调用旧 manager 的阻塞读取。状态监听者可以有多个，但运动命令应有清晰的
单一仲裁者。

## 常驻命令客户端

`HandCommandClient` 复用调用方已有的 Node、Publisher 和 DDS 连接：

```python
import rclpy
from rclpy.node import Node

from linkerbot_hand_ros.client import HandCommandClient


class ProductionNode(Node):
    def __init__(self):
        super().__init__("hand_business")
        self.hand = HandCommandClient(self, "/linkerbot_hand")

    def set_target(self, values):
        if not self.hand.is_ready("angles"):
            raise RuntimeError("hand lifecycle driver is not active")
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

`send_angles/speeds/torques` 默认只执行一次非阻塞 ROS 发布。驱动 active 时才创建命令
订阅者，因此 `is_ready()` 同时检查元数据、SDK 关节顺序和 DDS 订阅者存在性。

不要在控制循环中反复创建 Node、Publisher，或反复执行 `ros2 run`、
`ros2 topic pub --once`。这些临时进程的启动和 DDS 发现耗时不是驱动命令延迟。

## latest-wins 控制

连续目标在两层保持最新：

1. ROS command QoS 为 keep-last depth 1。
2. SDK `submit_*` 使用 `CommandPolicy.LATEST`，命令槽容量为 1。

当 1 kHz 业务循环驱动 100 Hz transport 时，中间旧目标会进入 `superseded`，机械手
不会追赶历史队列。重连也不会重放旧目标。

高频循环不要逐帧等待 DDS ACK。只有低频管理操作确实需要确认 ROS 可靠传输时才使用：

```python
receipt = self.hand.send_angles(
    [100, 100, 100, 100, 100, 100],
    wait_for_dds_ack=True,
    timeout_sec=1.0,
)
```

## 四层确认

| 层级 | 可观察接口 | 准确含义 |
| --- | --- | --- |
| DDS 传输 | `CommandReceipt.dds_acknowledged` | ROS 订阅端完成可靠传输 |
| SDK 接受 | `command/accepted/*`、`last_accepted()` | 参数有效并进入 SDK 调度器 |
| transport / 协议 | `/diagnostics` 的 `latest_*_stage` | 已发送，或设备协议明确 ACK |
| 机械到位 | 业务层状态窗口 | 新鲜反馈进入误差并稳定足够时间 |

这四层不能互相替代。L6 没有设备 ACK，成功终态为 `dispatched`；O30I 的连续命令可到
`acknowledged`，但仍不代表机械到位。

正式抓取流程至少应检查：

1. Lifecycle 驱动 active 且诊断 `connected=true`。
2. 目标已被 SDK 接受，且没有进入 `failed`。
3. 角度样本属于当前 `connection_generation`，有效、未过期且全部数值有限。
4. 目标误差在允许范围内持续稳定时间。
5. 电流、故障、业务超时和独立急停均满足安全策略。

当前兼容话题没有携带 sample generation/valid/stale 字段；严谨的跨重连到位判断应同时
订阅 `/diagnostics`，在 generation 变化时清空旧目标和稳定窗口。后续强类型状态消息会
把这些字段直接放入消息。

## Lifecycle 管理

launch 默认自动 configure/activate。由系统生命周期管理器接管时设置
`autostart:=false`：

```bash
ros2 lifecycle set /linkerbot_hand configure
ros2 lifecycle set /linkerbot_hand activate
ros2 lifecycle set /linkerbot_hand deactivate
ros2 lifecycle set /linkerbot_hand cleanup
```

- configure 不触碰硬件，可用于先验证配置和能力。
- activate 失败时节点关闭候选连接，不留下第二个连接或 worker。
- deactivate 先撤销命令订阅者，再停止 SDK 轮询，但保留连接。
- cleanup/shutdown 即使停止轮询失败也会继续尝试关闭。
- active 直接 shutdown 同样会停止 SDK worker、关闭连接并销毁驱动实体。
- deactivate 失败会进入 Lifecycle errorprocessing；错误回调再次有界 stop/close，成功后
  回到 unconfigured，不让 ROS 状态与连接所有权分叉。

静态参数只允许在 unconfigured 状态修改。在线修改仅限各传感器 `*_hz`、
`publish_rate_hz` 和 `diagnostic_rate`；切换型号、左右手、transport、命令频率或恢复策略
必须执行 `deactivate -> cleanup -> set parameters -> configure -> activate`。

## 断线恢复

`auto_reconnect=true` 时 SDK 在自身 worker 中恢复。恢复不会重新执行断线前或断线期间的
角度、速度、扭矩或离散管理命令。`connection_generation` 增加后，业务节点应
清除旧到位窗口，等待新一代角度样本，再显式下发新目标。

## 监控字段

部署系统至少监控：

- Lifecycle 状态和诊断 level
- `connection_state`、`connected`、`connection_generation`
- `consecutive_errors`、`recent_error_code`
- `angles_age_s`、`pending_commands`、`superseded_commands`
- `latest_angles_stage` 和对应错误码

诊断不能替代硬急停、机械限位、电流保护和上层动作超时。
