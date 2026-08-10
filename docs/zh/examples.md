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

指定其他驱动节点：

```bash
ros2 run linkerbot_hand_ros angle_command \
  --driver-name /left_hand/linkerbot_hand \
  --values 0 0 0 0 0 0
```

示例会等待至少一个订阅者，发布命令并等待 DDS 可靠传输确认；若 5 秒内没有找到
驱动节点或没有收到传输确认，会返回错误，不会把未确认的命令报告为发送成功。

`ros2 topic pub --once` 每次都会创建临时 ROS 2 节点并等待 DDS 发现，单次调用的
进程耗时通常比已运行的发布节点更长。连续控制时应使用常驻发布节点；命令行单次
控制优先使用本示例，不能用 CLI 进程总耗时判断 CAN 或机械动作延迟。

## 常驻角度控制器

需要连续发送角度时，启动一次常驻控制器：

```bash
ros2 run linkerbot_hand_ros angle_controller
```

控制器显示关节顺序后，可持续输入空格或逗号分隔的角度值：

```text
angles> 100 50 100 100 100 20
angles> 100,50,100,100,100,100
```

控制器复用同一个 ROS 2 Publisher，每条成功命令会显示 DDS 确认耗时。输入
`quit`、`exit`，或按 `Ctrl-D` 退出。使用命名空间时指定驱动节点完整名称：

```bash
ros2 run linkerbot_hand_ros angle_controller \
  --driver-name /left_hand/linkerbot_hand
```

原有 `--topic` 和 `--joint-names-topic` 参数仍可使用，控制器会解析并校验它们是否
属于同一个驱动节点。

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
