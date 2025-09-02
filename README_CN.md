# 1. **概述**

灵心巧手，创造万物。

LinkerHand 灵巧手 ROS SDK 是由灵心巧手（北京）科技有限公司开发的一款软件工具，用于驱动其灵巧手系列产品，并提供功能示例。它支持多种设备（如笔记本、台式机、树莓派、Jetson 等），主要服务于人型机器人、工业自动化和科研院所等领域，适用于人型机器人、柔性化生产线、具身大模型训练和数据采集等场景。

**警告**

1. 请保持远离灵巧手活动范围，避免造成人身伤害或设备损坏。

2. 执行动作前请务必进行安全评估，以防止发生碰撞。

3. 请保护好灵巧手。

# 2. **版本说明**
V2.1.9
1、支持L6\L6P\O6灵巧手

V2.1.8
1. 修复偶发撞帧问题

V2.1.4
1. 新增支持L21版本LinkeerHand灵巧手

2. 新增支持矩阵式压力传感器

3. 支持单USB转CAN同时控制同型号双手，需要重新接线

4. 支持双USB转CAN同时控制同型号双手

5. 新增L10通过RML485接口控制（由于睿尔曼机械臂Python调用485接口BUG问题，暂时不可用）

V2.0.2
1. 支持L7/O7/L10/O10/L20/O20/L25版本LinkerHand灵巧手

2. 支持动捕手套发布的实时速度

3. 支持获取所有LinkerHand灵巧手压感数据

4. 支持设置LinkerHand灵巧手手指速度

5. 支持设置LinkerHand灵巧手最大扭矩阈值

6. 支持LinkerHand灵巧手L20设置电流阈值 L7/L10/L25暂不支持

7. 支持LinkerHand灵巧手获取版本

8. 支持获取LinkerHand灵巧手L20电流阈值 L7/L10/L25暂不支持

9. 支持获取LinkerHand灵巧手当前关节状态

10. 支持获取LinkerHand灵巧手当前速度阈值

11. 支持获取LinkerHand灵巧手当前最大扭矩阈值

12. 支持获取LinkerHand灵巧手电机当前实时温度

13. 支持获取LinkerHand灵巧手电机故障码

14. 支持LinkerHand灵巧手L20清除电机故障码 L7/L10/L25暂不支持

15. 支持LinkerHand灵巧手L25设置电机使能 L7/L10/L20暂不支持

16. 支持LinkerHand灵巧手L25设置电机失能 L7/L10/L20暂不支持

17. 支持LinkerHand灵巧手L10&L20范围转弧度和弧度topic带有控制速度

V1.3.4

1. 波形图由单手显示改为单/双手配置显示，通过修改该配置文件压感是否存在来控制

2. 解决接近感应波形图数据同行错误

3. 改变关闭CAN口逻辑，避免第二次启动后灵巧手部分传感器数据读取不出来

V1.3.3

1. GUI新增了压力传感器波形图

2. L10支持了设置速度和设置扭矩

V1.3.2

1. 新增了T24版本灵巧手的支持

V1.3.1

1. examples新增LinkerHand灵巧手状态值(弧度与范围)的获取

2. 新增PyBullet仿真环境

3. 新增GUI控制界面

# 3. 准备工作

## 3.1 系统与硬件需求

* 操作系统：Ubuntu20.04

* ROS版本：Noetic

* Python版本：V3.8.10

* 硬件接口：5v标准USB接口

## 3.2 下载

```python
$ mkdir -p Linker_Hand_SDK_ROS/src    #创建目录
$ cd Linker_Hand_SDK_ROS/src    #进入目录
$ git clone https://github.com/linker-bot/linkerhand-ros-sdk.git    #获取SDK
```

## 3.3 安装依赖与编译

```python
$ sudo apt install python3-can
$ cd Linker_Hand_SDK_ROS/src/linker_hand_sdk    #进入目录
$ pip install -r requirements.txt    #安装所需依赖
$ cd Linker_Hand_SDK_ROS # 回到工程目录
$ catkin_make    #编译和构建ROS包
```
## 3.4 配置ROS主从通讯

支持分布式计算和模块化开发，只在本终端生效，如不需要则忽略。树莓派设备已默认配置完成。

```shell
$ source /opt/ros/noetic/setup.bash
$ export ROS_MASTER_URI=http://<ROS Master IP>:11311
$ export ROS_IP=<本机IP>
$ export ROS_HOSTNAME=<本机IP>
```

# 4. 使用

## 4.1 修改setting.yaml配置文件

无论是真机运行还是仿真模拟，均需要先修改配置参数文件。

目前，ROS开发的图形界面控制示例，只能单独控制一只LinkerHand灵巧手。

```shell
$ cd Linker_Hand_SDK_ROS/src/linker_hand_sdk/linker_hand_sdk_ros/scripts/LinkerHand/config
$ sudo vim setting.yaml    #编辑配置文件
```

setting.yaml描述

```yaml
VERSION: 2.0.2 # 重构核心源码,支持动捕手套速度
LINKER_HAND:  # 手部配置信息
  LEFT_HAND:
    EXISTS: True # 是否存在左手，如果不存在，请修改为False
    TOUCH: True  # 是否有压力传感器，如果不存在，请修改为False
    JOINT: L7 # 左手关节数  L7 \ L10 \ L20 \ L25
    NAME: # 无论l10还是l20 joint name都是20个
      - joint41
      - joint42
      - joint43
      - joint44
      - joint45
      - joint46
      - joint47
      - joint48
      - joint49
      - joint50
      - joint51
      - joint52
      - joint53
      - joint54
      - joint55
      - joint56
      - joint57
      - joint58
      - joint59
      - joint60
  RIGHT_HAND:
    EXISTS: False # 是否存在右手
    TOUCH: True # 是否有压力传感器
    JOINT: L10 # 右手关节数量   L7 \ L10 \ L20 \ L25
    NAME:  # 无论l10还是l20 joint name都是20个
      - joint71
      - joint72
      - joint73
      - joint77
      - joint75
      - joint76
      - joint77
      - joint78
      - joint79
      - joint80
      - joint81
      - joint82
      - joint83
      - joint84
      - joint88
      - joint86
      - joint87
      - joint88
      - joint89
      - joint90
PASSWORD: "12345678" # 由于与can通讯，需要激活通讯接口用到系统管理员密码
```
修改launch文件进行参数配置
```shell
$ cd /Linker_Hand_SDK_ROS/src/linker_hand_sdk_ros/launch/
$ sudo vim linker_hand.launch    #启动左or右单手，按照注释编辑配置文件
$ sudo vim linker_hand_double.launch    #启动左右双手，按照注释编辑配置文件
# 如果报错
ERROR: cannot launch node of type [linker_hand_sdk_ros/linker_hand.py]: Cannot locate node of type [linker_hand.py] in package [linker_hand_sdk_ros]. Make sure file exists in package path and permission is set to executable (chmod +x)
# 需要给执行文件权限
$ sudo chmod a+x src/linkerhand-ros-sdk/linker_hand_sdk_ros/scripts/linker_hand.py
# 然后在执行
$ sudo vim linker_hand.launch    #启动左or右单手，按照注释编辑配置文件
$ sudo vim linker_hand_double.launch    #启动左右双手，按照注释编辑配置文
```
- linker_hand.launch
```html
<?xml version="1.0" encoding="utf-8"?>
<launch>
    <node pkg="linker_hand_sdk_ros" type="linker_hand.py" name="linker_hand_sdk" output="screen" >  <!--  启动SDK  -->
        <param name="hand_type" type="string" value="right"/> <!--left or right-->
        <param name="hand_joint" type="string" value="L10"/> <!--L7 | L10 | L20 | L21 | L25-->
        <param name="touch" type="bool" value="true"/> <!--是否有压力传感器-->
    </node>
</launch>
```

### 单USB转CAN控制双手控制 注：首先保证没有其他CAN设备接入控制电脑，将USB转CAN线按照同颜色接在一起即可 支持L20|L21|L25
- 修改linker_hand_double.launch
```html
    <arg name="left_hand_joint" default="L10"/> <!-- 左手型号  L20 | L21 | L25-->
    <arg name="right_hand_joint" default="L10"/> <!-- 右手型号  L20 | L21 | L25-->
    <arg name="left_touch" default="true"/> <!-- 左手压力传感器 true or false-->
    <arg name="right_touch" default="true"/> <!-- 右手压力传感器 true or false-->
    <arg name="left_can" default="can0"/> <!-- 左手USB转CAN编号 can0-->
    <arg name="right_can" default="can0"/> <!-- 右手USB转CAN编号 can0-->
```

### 双USB转CAN控制双手控制 注：首先保证没有其他CAN设备接入控制电脑，先插入左手USB转CAN为can0，再插入右手USB转CAN为can1 支持L7 | L10 | L20 | L21 | L25
- 修改linker_hand_double.launch
```html
    <arg name="left_hand_joint" default="L10"/> <!-- 左手型号 L7 | L10 | L20 | L21 | L25-->
    <arg name="right_hand_joint" default="L10"/> <!-- 右手型号 L7 | L10 | L20 | L21 | L25-->
    <arg name="left_touch" default="true"/> <!-- 左手压力传感器 true or false-->
    <arg name="right_touch" default="true"/> <!-- 右手压力传感器 true or false-->
    <arg name="left_can" default="can0"/> <!-- 左手USB转CAN编号 can0-->
    <arg name="right_can" default="can1"/> <!-- 右手USB转CAN编号 can0-->
```
## 4.2 LinkerHand灵巧手与电脑硬件连接

### 4.2.1 将LinkerHand灵巧手的USB转CAN设备接口，插入Ubuntu设备上，蓝灯亮起

![](https://lkaeimso7m.feishu.cn/space/api/box/stream/download/asynccode/?code=YzkzNzI4YmE1MzdmMzMzOGFhNmUyMDQ4YjViMDk0N2JfZGxnTHZMaU9Ec2dqdkZKOERBeTcxU3BScDN1QTdPcXJfVG9rZW46RkNZYWIxNWllb0NlMW94SkR5aGNhcEs0bm9oXzE3NDM1ODU1MDk6MTc0MzU4OTEwOV9WNA)

灯光闪烁：蓝色灯闪烁代表通讯成功。

通讯模式：如何通过CAN模块连接的时候，区分是CAN还是485？ CAN接口和485接口完全不同。连接方式也不同

## 4.3 启动SDK

启动LinkerHand L10、L20灵巧手SDK，启动成功后会有SDK版本、CAN接口状态、灵巧手配置信息和当前灵巧手关节速度等提示信息。&#x20;

```shell
# 开启CAN端口
$ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 #USB转CAN设备蓝色灯常亮状态 在按照要求修改setting.ymal配置文件后，Ubuntu系统可以不做此步
$ cd ~/Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch #左or右单手启动
or
$ roslaunch linker_hand_sdk_ros linker_hand_double.launch #启动左右双手
# 如果报错
ERROR: cannot launch node of type [linker_hand_sdk_ros/linker_hand.py]: Cannot locate node of type [linker_hand.py] in package [linker_hand_sdk_ros]. Make sure file exists in package path and permission is set to executable (chmod +x)
# 需要给执行文件权限
$ sudo chmod a+x src/linker_hand_sdk/linker_hand_sdk_ros/scripts/linker_hand.py
$ roslaunch linker_hand_sdk_ros linker_hand.launch
```

### 4.4 通过RML机械臂485接口控制L10灵巧手 注：睿尔曼的官方API2只支持Python3.9以上版本，否则无法使用
- 修改linker_hand.launch文件
```bash
修改参数 <arg name="modbus" default="None"/> <!-- None or RML  only L10 --> 
# 开启CAN端口
$ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 #USB转CAN设备蓝色灯常亮状态 在按照要求修改setting.ymal配置文件后，Ubuntu系统可以不做此步
$ cd ~/Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch #左or右单手启动
向topic /cb_left_hand_control_cmd or /cb_right_hand_control_cmd 发送控制消息即可
```

- position与手指关节对照表
  ```bash
  $ rostopic echo /cb_left_hand_control_cmd
  ```
  ```bash
  header: 
    seq: 256
    stamp: 
      secs: 1744343699
      nsecs: 232647418
    frame_id: ''
  name: []
  position: [155.0, 162.0, 176.0, 125.0, 255.0, 255.0, 180.0, 179.0, 181.0, 68.0]
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  ```
  L7:  ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲","拇指旋转"]

  L10: ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部","小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"]

  L20: ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端","中指末端","无名指末端","小指末端"]

  L21: ["大拇指根部", "食指根部", "中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","预留","预留","预留","预留","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

  L25: ["大拇指根部", "食指根部", "中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部","中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]

# 5. **功能包介绍**

## 5.1 linker_hand_sdk_ros

控制灵巧手关节角度、获取灵巧手的各种状态信息。


## 5.2 examples

包含了各个产品的使用案例

## 5.3 doc

文档附件目录

# 6. ROS开发示例

## 6.1 PyBullet仿真

已支持的LinkerHand灵巧手产品：L10、L20、L25

1. 开启一个新终端，启动ROS

```shell
$ roscore
```

* 开启一个新终端，启动PyBullet仿真

```shell
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun linker_hand_pybullet linker_hand_pybullet.py _hand_type:=L20
```

**&#x20;参数说明**

"_hand_type"数值，无默认，必须根据产品型号选择，例如：_hand_type:=L20

**输出结果示例**

无

## 6.2 **图形界面控制**

已支持的LinkerHand灵巧手产品：L7/O7/L10/O10/L20/O20/L25/O25/T25
图形界面可控制[Mujoco仿真和PyBullet仿真](https://github.com/linkerbotai/linker_hand_sim)内的灵巧手

图形界面控制可以通过滑动块控制LinkerHand灵巧手L7/O7/L10/O10/L20/O20/L25/O25/T25各个关节独立运动。也可以通过添加按钮记录当前所有滑动块的数值，保存LinkerHand灵巧手当前各个关节运动状态。通过功能性按钮进行动作复现。

使用gui_control控制LinkerHand灵巧手: gui_control界面控制灵巧手需要启动linker_hand_sdk_ros，以topic的形式对LinkerHand灵巧手进行操作。
1. 开启一个新终端，启动ROS

```shell
$ roscore
```

* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动灵巧手 SDK
```

* 开启一个新终端，启动图形界面控制

```shell
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ roslaunch gui_control gui_control_left.launch # 控制左手，需要修改launch文件内的配置参数，按照自身需求即可
or
$ roslaunch gui_control gui_control_right.launch # 控制右手，需要修改launch文件内的配置参数，按照自身需求即可
or 
$ roslaunch gui_control gui_control_double.launch # 控制双手，需要修改launch文件内的配置参数，按照自身需求即可
```

开启后会弹出UI界面。通过滑动条可控制相应LinkerHand灵巧手关节运动。并可通过右侧添加按钮对当前滑动条数据进行保存，以便用于复现使用。

**参数说明**

无

**输出结果示例**

无

## 6.3 获取设备状态与信息
```bash
# 启动SDK后
$ rostopic echo /cb_left_hand_info

data: "{\"version\": [10, 6, 67, 76, 35, 18, 0], \"hand_joint\": \"L10\", \"speed\": [119,\
  \ 178, 178, 178, 178, -1, -1, -1, -1, -1], \"current\": [null, null, null, null,\
  \ null, null, null, null, null, null], \"fault\": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
  \ \"motor_temperature\": [32, 35, 34, 31, 34, 33, 34, 36, 33, 38], \"is_touch\"\
  : true, \"touch_type\": 2, \"finger_order\": []}"
```
**参数说明**
version: 版本编号
hand_joit: 灵巧手型号
speed: 当前速度阈值
current: 当前电流 null为暂不支持
fault: 对应电机错误码 0为正常
motor_temperature: 对应电机温度
is_touch: 是否支持压力传感器
touch_type: 压感模式，1、法向压力传感器。 2、矩阵压力传感器

## 6.4 设置

### 6.4.1 设置速度

已支持的LinkerHand灵巧手产品：L10、L20


* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动灵巧手
```

* 开启一个新终端，设置速度

```shell
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun set_linker_hand_speed set_linker_hand_speed.py _hand_type:=left _speed:=[180,250,250,250,250] # L7为7个值，其他为5个值
```

**参数说明**

L10、L20：5个手指统一的速度

speed:=\[180,250,250,250,250]

L7：7个电机速度

speed:=\[180,250,250,250,250,250,250]&#x20;

**输出结果示例**

speed:\[180,250,250,250,250]

### 6.4.2 设置当前电流

已支持的LinkerHand灵巧手产品：L10、L20


* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动灵巧手
```

* 开启一个新终端，设置当前电流

```shell
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun set_linker_hand_current set_linker_hand_current.py _hand_type:=left _current:=42 #暂不支持L7
```

**参数说明**

参数：hand_type: left | right 左手还是右手    current:0\~255 设置最大电流值

**输出结果示例**

current:\[180,250,250,250,250]

### 6.4.3 设置扭矩

已支持的LinkerHand灵巧手产品：L10、L20


* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动灵巧手
```

* 开启一个新终端，设置扭矩

```shell
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun set_linker_hand_torque set_linker_hand_torque.py _hand_type:=left _torque:=[180,250,250,250,250] # L7为7个值，其他为5个值
```

**参数说明**

参数：hand_type: left | right 左手还是右手    torque:=\[180,250,250,250,250]  0\~255 L7为7个电机最大值，其他为5个手指最大值&#x20;

**输出结果示例**

torque:\[180,250,250,250,250]

### 6.4.4 **设置为失能模式&#x20;**

已支持的LinkerHand灵巧手产品：L25

使灵巧手电机失能，可随意拖动各个关节活动


* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand_l25.launch # 启动L25
```

* 开启一个新终端，启动设置为失能模式

```shell
$ Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/L25
$ python set_disability.py
```

**参数说明**

无

**输出结果示例**

无

### 6.4.5 **设置为使能模式**

已支持的LinkerHand灵巧手产品：L25

使灵巧手电机使能，使能后，可用控制程序控制

1. 开启一个新终端，启动ROS

```css
$ roscore
```

* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动
```

* 开启一个新终端，启动设置为使能模式

```shell
$ Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/L25
$ python set_enable.py
```

**参数说明**

无

**输出结果示例**

无

### 6.4.6 **设置为遥操模式**

已支持的LinkerHand灵巧手产品：L25

如果拥有多只相同版本L25灵巧手，可使用本示例完成：一只失能的灵巧手控制另一只使能的灵巧手

首先需要启动LinkerHand SDK ROS 以下为被控制L25灵巧手配置方式，以下以右手为例 首先确保两台Ubuntu在同一网络内，并且配置好主从，两台Ubuntu可同时进行ROS通讯，可参考[ROS官方文档](https://wiki.ros.org/)&#x20;

**控制方-A灵巧手配置**


* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动
```

* 开启一个新终端，启动执行遥控

```shell
$ Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/L25
$ python set_remote_control.py
```

**被控制方-B灵巧手配置**


* 开启一个新终端

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch
```

此时，手动拖拽A机器的失能L25灵巧手即可控制B机器的使能L25灵巧手。

## 6.5 **应用演示**

### 6.5.1 **猜拳游戏**

已支持的LinkerHand灵巧手产品：L10、L20

注：需要有RGB摄像头


* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动灵巧手
```

* 开启一个新终端，启动猜拳游戏

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ rosrun finger_guessing finger_guessing.py
```

**参数说明**

无

**输出结果示例**

无

### 6.5.2 **捏取操作**

已支持的LinkerHand灵巧手产品：L20


* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动灵巧手
```

* 开启一个新终端，启动捏取演示

```python
python ./<你的文件路径>/lipcontroller.py
```

如终端打印出“**开始演示**”即为正常运行，此时手设置如果正确，应开始使用食指和中指进行捏的动作，捏到物品会停止，拿走物品后会继续尝试捏，直到捏到物品或运动到极限。

**lipcontroller.py** 是基于产品O7的第7版进行开发的演示demo，应用在其他版本的演示时，需要调整拇指和食指的对合姿态，否则无法实现“**食指和拇指捏合在一起**”的动作。

**参数说明**

无

**输出结果示例**

无

## 6.6 **模仿学习**

### 6.6.1 **模仿学习训练**

使用本例硬件为LinkerRobot人形机器人，也可以使用其他机械臂或机器人进行模仿学习训练，只要修改该相应数据话题即可， [详细使用说明请参考human-dex项目README.md](https://github.com/linkerbotai/human-dex)

1. 配置环境

```shell
cd human-dex
conda create -n human-dex python=3.8.10
conda activate human-dex
pip install torchvision
pip install torch
pip install -r requirements.txt
```

* 安装

```css
mkdir -p your_ws/src
cd your_ws/src
git clone https://github.com/linker-bot/human-dex.git
cd ..
catkin_make
source ./devel/setup.bash
```

* 运行

```shell
# 数据采集
 roslaunch record_hdf5 record_hdf5.launch
# 新开终端发送采集命令
rostopic pub /record_hdf5 std_msgs/String "data: '{\"method\":\"start\",\"type\":\"humanplus\"}'"
```

* 训练

```solidity
cd humanplus/scripts/utils/HIT
python3 imitate_episodes_h1_train.py --task_name data_cb_grasp --ckpt_dir cb_grasp/ --policy_class HIT --chunk_size 50 --hidden_dim 512 --batch_size 48 --dim_feedforward 512 --lr 1e-5 --seed 0 --num_steps 100000 --eval_every 1000 --validate_every 1000 --save_every 1000 --no_encoder --backbone resnet18 --same_backbones --use_pos_embd_image 1 --use_pos_embd_action 1 --dec_layers 6 --gpu_id 0 --feature_loss_weight 0.005 --use_mask --data_aug
```

* 复现/评估

```css
cd humanplus/scripts
python3 cb.py
```

**参数说明**

无

**输出结果示例**

无

### 6.6.2 **Unidexgrasp抓取算法**

原Unidexgrasp算法采用shadowhand，以下提供在linkerhand上开发Unidexgrasp算法的相关代码。 [详细过程参考linker_unidexgrasp项目](https://github.com/linkerbotai/linker_unidexgrasp)

**抓取姿态生成部分**

抓取姿态部分采取映射方案，将模型输出的shadowhand手姿，映射为linkerHand L20手姿态，为后续开发使用。

1. 配置环境

```bash
conda create -n unidexgrasp python=3.8
conda activate unidexgrasp
conda install -y pytorch==1.10.0 torchvision==0.11.0 torchaudio==0.10.0 cudatoolkit=11.3 -c pytorch -c conda-forge
conda install -y https://mirrors.bfsu.edu.cn/anaconda/cloud/pytorch3d/linux-64/pytorch3d-0.6.2-py38_cu113_pyt1100.tar.bz2
pip install -r requirements.txt
cd thirdparty/pytorch_kinematics
pip install -e .
cd ../nflows
pip install -e .
cd ../
git clone https://github.com/wrc042/CSDF.git
cd CSDF
pip install -e .
cd ../../
```

* 训练&#x20;

  1. GraspIPDF

  ```bash
  python ./network/train.py --config-name ipdf_config \
                            --exp-dir ./ipdf_train
  ```

  * GraspGlow

  ```bash
  python ./network/train.py --config-name glow_config \
                            --exp-dir ./glow_train
  python ./network/train.py --config-name glow_joint_config \
                            --exp-dir ./glow_train
  ```

  * ContactNet

  ```bash
  python ./network/train.py --config-name cm_net_config \
                            --exp-dir ./cm_net_train
  ```

* 验证

```bash
python ./network/eval.py  --config-name eval_config \
                          --exp-dir=./eval
```

* 映射，结果可视化

```bash
python ./tests/visualize_result_l20_shadow.py --exp_dir 'eval' --num 3
```

* 保存结果为后续强化学习算法开发使用

```bash
python ./tests/data_for_RL.py
```

# 7. Python开发示例

### 7.1 **手做OK手势**

已支持的LinkerHand灵巧手产品：L20

1. 开启一个新终端，启动ROS

```shell
$ roscore
```

* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动灵巧手
```

* 开启一个新终端，启动OK手势

```shell
python ./<你的文件路径>/gesture-Show-OK.py
```

开始后终端会打印测试中，此时手会开始做OK手势，并弯曲中指无名指小指和伸直动作。

**参数说明**

无

**输出结果示例**

无

### 7.2 **手做旋转食指动作**

已支持的LinkerHand灵巧手产品：L20

1. 开启一个新终端，启动ROS

```shell
$ roscore
```

* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动灵巧手
```

* 开启一个新终端，启动旋转食指动作

```shell
python ./<你的文件路径>/gesture-Show-Surround-Index-Finger.py
```

开始后终端会打印测试中，此时手会开始握拳并伸出食指，食指会不断重复旋转。

**参数说明**

无

**输出结果示例**

无

### 7.3 **手做波浪运动**

已支持的LinkerHand灵巧手产品：L20

1. 开启一个新终端，启动ROS

```shell
$ roscore
```

* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动灵巧手
```

* 开启一个新终端，启动波浪运动

```shell
python ./<你的文件路径>/gesture-Show-Wave.p
```

开始后终端会打印测试中，此时手拇指向外舒展不动，其余四指开始做波浪运动。

**参数说明**

无

**输出结果示例**

无

### 7.4 **手做一套复杂动作**

已支持的LinkerHand灵巧手产品：L20

1. 开启一个新终端，启动ROS

```shell
$ roscore
```

* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动灵巧手
```

* 开启一个新终端，启动一套复杂动作

```shell
python ./<你的文件路径>/gesture-Show-Ye.py
```

开始后终端会打印测试中，此时手会开始做一套复杂动作来展示手的灵活性。

本例是基于产品O7的第7版进行开发的演示demo，应用在其他版本的演示时，需要调整拇指和食指的对合姿态，否则无法实现“**食指和拇指捏合或对合在一起**”的动作。

**参数说明**

无

**输出结果示例**

无

### 7.5 **手做循环抓握动作**

已支持的LinkerHand灵巧手产品：L20

1. 开启一个新终端，启动ROS

```shell
$ roscore
```

* 开启一个新终端，启动Linker Hand ROS SDK

```shell
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # 启动 L10 or L20 灵巧手
```

* 开启一个新终端，启动循环抓握动作

```shell
$ cd Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/gesture-show
$ python gesture-Loop.py 
```

**参数说明**

无

**输出结果示例**

无

### 7.6 手指舞

已支持的LinkerHand灵巧手产品：L25

**参数说明**

无

**输出结果示例**

无

# 8. 相关GitHub资源

Human-Dex：https://github.com/linkerbotai/human-dex

Linker_UniDexGrasp：https://github.com/linkerbotai/linker_unidexgrasp

LinkerHand-Python-SDK：https://github.com/linkerbotai/linker_hand_python_sdk

linker_serl：https://github.com/linkerbotai/linker_serl

# 9. 常见问题

## 9.1 安装CAN模块驱动

```python
pip install python-can
```

## 9.2 如何无需重复输入密码？

方法一

```shell
$ sudo visudo
#添加以下内容
你的用户名 ALL=(ALL) NOPASSWD: /sbin/ip
你的用户名 ALL=(ALL) NOPASSWD: /usr/sbin/ip link set can0 up type $ $ can bitrate 1000000
# 保存退出
```

方法二

修改setting.yaml配置文件的密码，默认PASSWORD："12345678"


## 使用 PyBullet 模拟 L7\L10\L20\L21 仿真环境
[PyBullet Readme](https://github.com/linkerbotai/linker_hand_sim/blob/main/linker_hand_pybullet_ros/README_PyBullet_CN.md)

## 使用 Mujoco 模拟 L7\L10\L20\L21 仿真环境
[Mujoco Readme](https://github.com/linkerbotai/linker_hand_sim/blob/main/linker_hand_mujoco_ros/README_CN.md)

## 使用 IsaacGym 模拟 L20 仿真环境
[IsaacGym Readme](examples/L20/l20_isaacgym/isaacgym_l20.md)

## Topic Document
[Linker Hand Topic Document](doc/Topic-Reference.md)

