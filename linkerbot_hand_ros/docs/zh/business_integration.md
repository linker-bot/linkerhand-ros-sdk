# 业务节点集成

## 唯一集成路径

业务节点只依赖类型化 ROS interface，不导入型号类、不维护 `MODEL_SPECS`，也不主动发送
设备查询。推荐依赖关系：

```text
业务控制器
  -> HandCommandClient + linkerbot_hand_msgs
  -> linkerbot_hand Lifecycle 驱动
  -> HandDevice 1.x polling/scheduler/cache
  -> linkerbot-transport
```

业务启动时订阅 transient-local `capabilities`，从中确定：

- verified 型号、固件、side 和 sensor variant；
- 关节名称、顺序和数量；
- percent endpoint 含义、协议 raw 范围和整数约束；
- 支持的命令、telemetry、管理操作与 ACK 能力。

不要在业务包中复制型号范围表。activate 后 capability 会从 declared 更新为 verified，只有
`identity.verified=true` 才允许进入运动状态机。

## 控制循环

`HandCommandClient` 复用 publisher、capability 和 status subscription：

```python
from linkerbot_hand_ros.client import HandCommandClient

self.hand = HandCommandClient(self, "/linkerbot_hand")

if self.hand.is_ready("angles"):
    publication = self.hand.set_percent_angles(target_percent)
    self.pending_user_sequence = publication.user_sequence
```

这一步只表示 DDS publish 完成。命令状态从 `/command/status` 异步到达。高频 timer/callback
中不得调用 `wait_for_terminal_status()`，否则同一 executor 可能无法处理 status。使用状态机：

```text
publish target
  -> wait ACCEPTED/DISPATCHED/ACK asynchronously
  -> observe current-generation angle samples
  -> apply tolerance + settle duration
  -> declare mechanical arrival
```

连续角度使用 best-effort depth 1 和 SDK latest-wins。控制器应按最新目标设计，不能假设每个
历史 setpoint 都会发送。`SUPERSEDED` 表示旧目标在 transport claim 前确认未发送。

## 状态与到位判断

订阅 `HandJointState` 后至少检查：

1. `meta.valid=true` 且 `meta.stale=false`。
2. `meta.connection_generation` 等于当前驱动 generation。
3. `joint_names` 与当前 verified capability 完全一致。
4. 需要 percent 的轴其 `percent_valid=true`；需要物理弧度的轴其
   `physical_rad_valid=true`。
5. `meta.sequence` 相对上一样本增加，不能把重复 ROS callback 当作新设备数据。

驱动本身只在 sequence 或有效性签名变化时发布，因此正常情况下不会重复同一缓存。业务仍应
保存 sequence，用于检测乱序、代次切换和长时间无新样本。

以下阶段严格区分：

| 层次 | 观察方式 | 能证明什么 |
| --- | --- | --- |
| ROS 发布 | `CommandPublication` | 业务消息已交给本地 DDS publisher |
| SDK 接受 | `HandCommandStatus.ACCEPTED` | 参数有效并进入 SDK 调度器 |
| transport 发送 | `DISPATCHED` | 全部请求帧实际 send 成功 |
| 设备协议确认 | `ACKNOWLEDGED` | 协议明确 ACK，只有支持型号才有 |
| 机械到位 | 新鲜角度反馈窗口 | 误差和稳定时间满足业务判据 |

## Percent、raw 和物理角度

- `set_percent_angles()` 对应 `HandJointCommand.REPRESENTATION_PERCENT`。
- `set_angles()` 对应 `REPRESENTATION_RAW`，是精确协议值。
- `HandJointState` 同时提供 raw、percent 和可选 physical radians，有独立 validity mask。

普通 `set_*` 和 telemetry `raw` 不做 RPM、mA 或 N.m 转换。configured speed 不等于
measured speed，current limit 不等于 torque。L6 的速度设置快照只能作为
`configured_speeds`，不能用于实际速度闭环。

## 错误处理

服务响应和命令 status 提供稳定 `error_code`，例如：

- `validation`
- `unsupported_operation`
- `unsupported_firmware`
- `identity_mismatch`
- `device_not_ready`
- `transport`
- `protocol`
- `timeout`
- `device_fault`
- `queue_full`
- `polling_plan`
- `closed`
- `lifecycle`

业务代码按 code 分支，不解析中英文 message。`unsupported_operation` 是能力差异，不应一律
升级为系统故障；`identity_mismatch`、重复 transport 错误和设备 fault 应进入安全状态。

## Lifecycle 协调

启动顺序：

1. configure 驱动并读取 declared capability。
2. activate，等待 verified capability、device info、diagnostics operational。
3. 等待当前 generation 的有效角度 baseline。
4. 启用业务命令发布。

停止顺序：

1. 业务节点停止产生新目标。
2. deactivate 驱动，撤销命令入口并停止 polling。
3. cleanup 或 shutdown 关闭连接。

通用驱动不会在 deactivate 自动发送保持、全开或握拳。需要停止动作时由上层安全控制器定义
明确目标和超时，不能依赖生命周期隐式动作。

## 断线和恢复

看到 disconnected/reconnecting 或 generation 改变时：

- 立即停止提交目标，清空 pending 业务状态和到位稳定窗口；
- 不缓存“等恢复后发送”的运动目标；
- 等待 SDK 重新验证身份和固件，读取 baseline；
- 检查 fault 和安全条件；
- 由业务显式提交新目标。

SDK 禁止重放断线前目标，也不会自动清 fault、enable 或 calibrate。业务层同样不能把旧
`user_sequence` 的 status 用于新 generation。

## 多设备部署

每个设备使用独立 namespace，例如 `/left/linkerbot_hand` 和
`/right/linkerbot_hand`。使用 `multi_hand.launch.py` 和一个参数文件把这些 Lifecycle 节点
放入同一进程；不要分别启动 `hand.launch.py`。相同物理 transport key 的节点会共享一个
runtime、RX reactor 和 TX arbiter；不同 channel 不共享物理 runtime。

容器自动启动采用全量 configure 后全量 activate，任何失败都会逆序回滚，业务不会看到容器
留下部分 active 设备。具体参数文件见[Lifecycle 与参数配置](configuration.md#多设备)。

业务层分别持有两个 `HandCommandClient`，按各自 capability 和 generation 管理，不假设两种
型号具有相同关节数量或相同 telemetry。
