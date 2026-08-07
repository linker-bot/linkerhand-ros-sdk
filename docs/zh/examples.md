# 使用示例

以下命令默认节点名为 `/linkerbot_hand`。如果使用了命名空间，需要相应修改话题名。

## 查看接口与关节顺序

```bash
ros2 node info /linkerbot_hand
ros2 topic echo --once /linkerbot_hand/joint_names
ros2 topic hz /linkerbot_hand/state/angles
```

## L6 张开和握拳方向

```bash
# 以较低速度运动
ros2 topic pub --once /linkerbot_hand/command/speeds \
  std_msgs/msg/Float64MultiArray "{data: [20, 20, 20, 20, 20, 20]}"

# 张开；数组顺序必须与 joint_names 一致
ros2 topic pub --once /linkerbot_hand/command/angles \
  std_msgs/msg/Float64MultiArray "{data: [100, 50, 100, 100, 100, 100]}"

# 向握拳方向运动，但保留端点余量
ros2 topic pub --once /linkerbot_hand/command/angles \
  std_msgs/msg/Float64MultiArray "{data: [20, 20, 20, 20, 20, 20]}"
```

对 L6 而言，屈伸关节的 `0` 是握拳方向、`100` 是张开方向。拇指侧摆的典型
张开目标不是 100，SDK 示例使用 50。首次实机测试不要直接命令 0 或 100 极限。

## L30 设置速度、扭矩和角度

L30 有 17 个关节，速度范围为 1–250，扭矩范围为 60–800。L30 的百分比按
每个关节的协议最小/最大值映射，下面的 0 仅表示协议最小值，不统一等同于张开：

```bash
ros2 topic pub --once /linkerbot_hand/command/speeds \
  std_msgs/msg/Float64MultiArray \
  "{data: [30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30]}"

ros2 topic pub --once /linkerbot_hand/command/torques \
  std_msgs/msg/Float64MultiArray \
  "{data: [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100]}"

ros2 topic pub --once /linkerbot_hand/command/angles \
  std_msgs/msg/Float64MultiArray \
  "{data: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

## Python 角度命令示例

安装包后可直接运行 [`angle_command.py`](../../linkerbot_hand_ros/examples/angle_command.py)：

```bash
ros2 run linkerbot_hand_ros angle_command --values 100 50 100 100 100 100
```

指定其他话题：

```bash
ros2 run linkerbot_hand_ros angle_command \
  --topic /left_hand/linkerbot_hand/command/angles \
  --values 0 0 0 0 0 0
```

示例会等待至少一个订阅者后发布一次命令；若 5 秒内没有找到驱动节点，会返回错误。

## Python 状态监听示例

运行 [`state_monitor.py`](../../linkerbot_hand_ros/examples/state_monitor.py)：

```bash
ros2 run linkerbot_hand_ros state_monitor
```

监听命名空间内的节点：

```bash
ros2 run linkerbot_hand_ros state_monitor \
  --names-topic /left_hand/linkerbot_hand/joint_names \
  --angles-topic /left_hand/linkerbot_hand/state/angles
```

程序会先接收逗号分隔的关节名，然后逐行输出 `关节名=角度`。如果角度数组长度
与关节名不一致，会输出明确告警。默认最多每 0.5 秒输出一次，可通过
`--print-interval` 调整。
