# 快速入门

## 1. 安装依赖

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install 'linkerbot-hand>=0.1.0,<0.2.0'
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep can-utils
```

使用 SDK 本地源码时：

```bash
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-transport.git
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand.git
python3 -m pip install -e ./linkerbot-transport -e ./linkerbot-hand
python3 -c 'import linkerbot_hand; print(linkerbot_hand.__file__)'
```

## 2. 准备 CAN 接口

O6、L6、L20Lite 和 L25 使用经典 CAN。实际波特率以硬件手册为准：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

O20、O30I 和 L30 使用 CAN FD：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can \
  bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
ip -details -statistics link show can0
```

`candump can0` 会占用并显示总线帧，排查结束后按 `Ctrl-C` 退出。

## 3. 构建工作空间

```bash
mkdir -p ~/linkerbot_ws/src
cd ~/linkerbot_ws/src
git clone ssh://git@git.linkerhub.work:12222/zhuzhixiang/linkerbot-hand-ros.git
cd ~/linkerbot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/linkerbot_ws/install/setup.bash
```

## 4. 启动 Lifecycle 驱动

L6 示例：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left \
  transport_options_json:='{"interface_name":"can0","interface_type":"socketcan"}'
```

L30 SocketCAN 示例：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l30 side:=unknown \
  transport_options_json:='{"interface_type":"socketcan","channel":"can0","node_id":1}'
```

O20 和 O30I 的 SocketCAN 通道键为 `socketcan_channel`。完整构造参数应以相同版本
`linkerbot-hand` SDK 的型号文档为准，ROS 不再复制维护这些构造器规则。

launch 默认 `autostart:=true`。由外部生命周期管理器接管时使用：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py autostart:=false
ros2 lifecycle set /linkerbot_hand configure
ros2 lifecycle set /linkerbot_hand activate
```

## 5. 先验证状态，不发送运动命令

```bash
ros2 lifecycle get /linkerbot_hand
ros2 node info /linkerbot_hand
ros2 topic echo --once /linkerbot_hand/joint_names
ros2 topic echo --once /linkerbot_hand/model
ros2 topic echo --once /linkerbot_hand/state/angles
ros2 topic echo --once /diagnostics
```

`read_now` 只把 SDK 中有效且未过期的角度缓存立即发布一次，不会增加 CAN 查询：

```bash
ros2 service call /linkerbot_hand/read_now std_srvs/srv/Trigger '{}'
```

## 6. 使用常驻客户端测试 L6

先确认急停、机械空间、速度和扭矩安全，再启动：

```bash
ros2 run linkerbot_hand_ros angle_controller
```

建议先从中间值做小范围测试。L6 的全 `100` 整手目标为完全张开、全 `0` 整手目标为
完全握拳；其中 `thumb_abd` 是方向轴，单独解释时 0/100 表示 capability 声明的位置
端点。首次实机验证不应直接从一个机械端点跳到另一个端点。
