# L6 业务化接口实机测试报告

> 历史报告：本文记录 0.2.0 旧 SDK 适配层的实测结果，不代表当前 Lifecycle、缓存轮询
> 和 `CommandHandle` 路径已完成实机验证。当前接口见[ROS 2 接口](../ros_api.md)。

## 测试环境

| 项目 | 值 |
| --- | --- |
| 日期 | 2026-08-10 |
| 系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| 软件版本 | `linkerbot_hand_ros 0.2.0` |
| 设备 | 左手 L6，`can0`，SocketCAN |
| 节点 | `/linkerbot_hand` |

驱动与测试进程均加载以下环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/linkerbot_ws/.venv/bin/activate
source ~/linkerbot_ws/install/setup.bash
```

## 自动化检查

- Pytest：42 项全部通过
- 定向 Flake8：通过
- Python `compileall`：通过
- `ament_xmllint package.xml`：通过
- 隔离 `colcon build --symlink-install`：通过
- 隔离 `colcon test`：43 项全部通过

## ROS 2 接口

后启动的订阅者可立即读到 transient-local 元数据：

```text
model: l6
joint_names: thumb_flex,thumb_abd,index,middle,ring,pinky
```

角度命令订阅端实测为 `RELIABLE + VOLATILE`，SDK 接受反馈发布端为
`RELIABLE + TRANSIENT_LOCAL`。`/diagnostics` 正常发布标准
`diagnostic_msgs/msg/DiagnosticArray`。

## 常驻客户端与运动结果

测试程序只创建一个 `HandCommandClient`，在同一 ROS 2 连接中依次发送目标位和
恢复位：

| 操作 | 结果 |
| --- | --- |
| 非阻塞发布 `[20, 50, 20, 20, 20, 20]` | 调用耗时 113.769 us |
| SDK accepted 反馈 | 2.075 ms，数值与目标一致 |
| 目标位状态读回 | `[20.39, 50.59, 19.61, 20.00, 19.61, 19.61]` |
| 恢复全 100 并等待 DDS ACK | ACK 成功，总耗时 5.526 ms |
| 恢复命令 accepted 反馈 | 6.509 ms，数值为全 100 |
| 最终稳定状态 | `[99.61, 99.61, 99.61, 99.61, 99.61, 100.00]` |

角度状态与目标的误差在设备离散分辨率范围内。L6 测试结束后保持全 100 张开。

## 诊断结果

运动测试结束后的诊断状态为 `operational`：

| 指标 | 值 |
| --- | --- |
| `state_successes` | 3171 |
| `state_errors` | 0 |
| `command_successes` | 2 |
| `command_errors` | 0 |
| `last_error` | 空 |

本次测试证明常驻客户端、最新目标 QoS、SDK 接受反馈、状态读回和标准诊断可在
同一 L6 驱动进程中协同工作。DDS ACK 和 accepted 均不代表机械到位，正式业务仍需
根据 `state/angles`、动作容差和超时判断最终结果。
