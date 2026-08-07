# 故障排查

## 找不到包或可执行文件

```text
Package 'linkerbot_hand_ros' not found
```

确认已构建并在当前终端加载工作空间：

```bash
cd ~/linkerbot_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg executables linkerbot_hand_ros
```

## Python 找不到 linkerbot

```text
ModuleNotFoundError: No module named 'linkerbot'
```

将 SDK 安装到运行 `ros2` 的同一个 Python 环境：

```bash
python3 -c 'import sys; print(sys.executable)'
python3 -m pip install 'linkerbot>=0.5.4'
python3 -c 'import linkerbot; print(linkerbot.__file__)'
```

## CAN 接口不存在或未启动

```bash
ip link show can0
ip -details -statistics link show can0
```

如果接口为 `DOWN`，按[快速入门](getting_started.md)重新配置。检查 USB-CAN 设备名
是否实际为 `can1` 或其他名称。

## 持续出现读取超时

依次检查：

1. `model`、`side` 和 `interface_name` 是否与硬件一致。
2. 经典 CAN 与 CAN FD 模式、仲裁段和数据段波特率是否正确。
3. 总线两端是否有正确的 120 Ω 终端电阻。
4. `candump can0` 是否能看到收发帧。
5. 总线负载较高时，将 `poll_rate` 降至 `10.0`、`optional_state_rate` 降至 `0.5`。
6. 将 `timeout_ms` 适当提高，例如 `200.0`。

## 命令被拒绝

日志中的 `rejected angles/speeds/torques command` 表示命令没有发送到硬件。常见原因：

- 数组长度不等于当前型号的关节数。
- 数值超出[型号命令范围](ros_api.md#命令范围)。
- O20、O30I、L30 的速度或扭矩使用了小数。

先执行以下命令确认顺序和数量：

```bash
ros2 topic echo --once /linkerbot_hand/joint_names
```

## 没有温度、电流或故障话题数据

不同型号暴露的传感器不同。确认 `publish_optional_state` 为 `true`，并使用：

```bash
ros2 param get /linkerbot_hand publish_optional_state
ros2 topic list | sort
```

节点会创建可选状态话题，但只有 SDK 和设备支持相应读取时才发布数据。

## 故障清除返回失败

```bash
ros2 service call /linkerbot_hand/clear_faults std_srvs/srv/Trigger '{}'
```

O30I 和 L30 当前只支持故障读取，不支持通过 SDK 清除。其他型号返回失败时，应先
排除过温、过流、机械卡滞或通信异常；不要反复清除仍在发生的故障。
