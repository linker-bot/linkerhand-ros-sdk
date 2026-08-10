# 快速入门

## 1. 安装依赖

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install 'linkerbot>=0.5.4'
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

如果尚未初始化 `rosdep`：

```bash
sudo rosdep init
rosdep update
```

## 2. 准备 CAN 接口

O6、L6、L20Lite 和 L25 使用经典 CAN。下面仅给出常见 SocketCAN 配置形式，
实际波特率以对应硬件手册为准：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

O20、O30I 和 L30 使用 CAN FD。SDK 默认配置为仲裁段 1 Mbit/s、数据段
5 Mbit/s：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can \
  bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
ip -details link show can0
```

可以用 `candump` 确认总线是否收到数据：

```bash
sudo apt install can-utils
candump can0
```

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

## 4. 启动节点

经典 CAN 型号：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l6 side:=left interface_name:=can0 interface_type:=socketcan
```

SocketCAN CAN FD 型号：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l30 interface_name:=can0 interface_type:=socketcan
```

使用厂商 CAN FD 动态库时，将后端设为 `ctypes` 并提供动态库路径：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py \
  model:=l30 interface_type:=ctypes \
  library_path:=/opt/linkerbot/lib/libcanbus.so
```

## 5. 验证通信

```bash
ros2 node info /linkerbot_hand
ros2 topic echo --once /linkerbot_hand/joint_names
ros2 service call /linkerbot_hand/read_now std_srvs/srv/Trigger '{}'
ros2 topic echo --once /linkerbot_hand/state/angles
```

确认状态可正常读取后，先设置低速度和保守扭矩，再从远离 0/100 端点的小范围
动作开始测试。角度增大或减小对应的物理方向必须按具体型号确认。
