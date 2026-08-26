# 从 0.2.x 迁移到 0.3.0

0.3.0 将驱动迁移到拆分后的 `linkerbot-hand` 和 ROS 2 Lifecycle。现有 Topic 名称
和 `HandCommandClient` 保持兼容，但启动、参数、状态来源和确认语义发生变化。

## 依赖

```bash
python3 -m pip install --upgrade \
  --index-url https://gitea.linkerhub.work/api/packages/zhuzhixiang/pypi/simple \
  --extra-index-url https://pypi.org/simple \
  'linkerbot-hand>=0.2.1,<0.3.0'
```

ROS 驱动只直接依赖独立发行的 `linkerbot-hand`，由 hand 传递依赖兼容的
`linkerbot-transport`；不依赖旧聚合 `linkerbot`、机械臂包或 `linkerbot-python-sdk` 仓库。

## 参数映射

| 0.2.x | 0.3.0 | 迁移说明 |
| --- | --- | --- |
| `poll_rate` | `angles_hz` | 设备角度查询由 SDK 执行 |
| 无 | `publish_rate_hz` | ROS 缓存发布频率，与设备查询解耦 |
| `publish_optional_state` | 删除 | 将不需要的传感器频率设为 0 |
| `optional_state_rate` | 各传感器 `*_hz` | 每种传感器独立配置，包括 voltage/acceleration/motion_time |
| `timeout_ms` | 删除 | 阻塞读取已从 ROS 路径移除 |
| 无 | `command_rate_hz` | SDK transport 连续命令限频 |
| 无 | `auto_reconnect` | SDK 恢复策略 |
| 三个 transport 参数 | `transport_options_json` | 新部署优先使用 JSON |

旧 `interface_name/interface_type/library_path` 暂时保留，只有 JSON 为 `{}` 时生效。

## 启动

推荐 launch，默认自动 configure/activate：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left \
  transport_options_json:='{"interface_name":"can0","interface_type":"socketcan"}'
```

直接 `ros2 run ... hand_node` 后节点停在 unconfigured，必须由生命周期管理器切换状态。

## 行为变化

- ROS 不再直接调用 `get_blocking()`，所有发布只读 SDK 缓存。
- `read_now` 不再查询设备，只立即发布新鲜缓存。
- inactive 状态没有运动命令订阅者。
- `command/accepted/*` 只对应 SDK `ACCEPTED`，不是 transport 已发送。
- 最近 `dispatched/acknowledged/failed/superseded` 阶段通过诊断观察。
- 屈伸轴统一为 `100=完全张开，0=完全握拳`；方向轴的 0/100 表示 capability
  声明的两个位置端点。
- 从 0.3.3 起，协议角度必须能通过 SDK capability 完成整组百分比映射，否则不发布
  `state/angles`。
- 自动重连不重放旧目标，generation 变化后业务必须显式下发新目标。
- 静态参数在 configure 后不再表面接受；切换连接配置必须先 cleanup。
- L30 的 enable/disable/calibrate 通过异步 Trigger 服务进入 SDK 离散 FIFO。
- 从 0.3.3 起，force 触觉矩阵通过带 `finger/row/column` 布局的
  `Float64MultiArray` 发布；时间、序号和连接代次仍从 diagnostics 获取。

## 上线核对

```bash
ros2 lifecycle get /linkerbot_hand
ros2 topic echo --once /linkerbot_hand/joint_names
ros2 topic echo --once /linkerbot_hand/state/angles
ros2 topic echo --once /diagnostics
```

先完成无运动状态验证，再用常驻客户端从中间值和低速度做小范围实机验证。0.2.x 历史
测试报告不能替代 0.3.0 新命令路径的硬件回归。
