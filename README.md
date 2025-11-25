<img  src="resource/logo.png" width="800">

# 1. **Overview**

Intelligent Dexterous Hands, Creating All Possibilities.

The LinkerHand ROS SDK is a software tool developed by LinkerHand (Beijing) Technology Co., Ltd. used to drive its series of dexterous hand products and provide functional examples. It supports various devices (such as laptops, desktops, Raspberry Pi, Jetson, etc.) and primarily serves fields like humanoid robotics, industrial automation, and scientific research institutions. It is suitable for applications such as humanoid robots, flexible production lines, embodied AI model training, and data collection.

[中文](README_CN.md)  |  [English](README.md)

**WARNING**

1. Please stay clear of the dexterous hand's working range to avoid personal injury or equipment damage.

2. Always conduct a safety assessment before executing any motion to prevent collisions.

3. Please protect the dexterous hand from damage.

| Name | Version | Link |
| --- | --- | --- |
| Python SDK | ![SDK Version](https://img.shields.io/badge/SDK%20Version-V3.0.1-brightgreen?style=flat-square) ![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue?style=flat-square&logo=python&logoColor=white) ![Windows 11](https://img.shields.io/badge/OS-Windows%2011-0078D4?style=flat-square&logo=windows&logoColor=white) ![Ubuntu 20.04+](https://img.shields.io/badge/OS-Ubuntu%2020.04%2B-E95420?style=flat-square&logo=ubuntu&logoColor=white) | [![GitHub 仓库](https://img.shields.io/badge/GitHub-grey?logo=github&style=flat-square)](https://github.com/linker-bot/linkerhand-python-sdk) |
| ROS SDK | ![SDK Version](https://img.shields.io/badge/SDK%20Version-V3.0.1-brightgreen?style=flat-square) ![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue?style=flat-square&logo=python&logoColor=white) ![Ubuntu 20.04+](https://img.shields.io/badge/OS-Ubuntu%2020.04%2B-E95420?style=flat-square&logo=ubuntu&logoColor=white) ![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-009624?style=flat-square&logo=ros) | [![GitHub 仓库](https://img.shields.io/badge/GitHub-grey?logo=github&style=flat-square)](https://github.com/linker-bot/linkerhand-ros-sdk) |
| ROS2 SDK | ![SDK Version](https://img.shields.io/badge/SDK%20Version-V3.0.1-brightgreen?style=flat-square) ![Python 3.11](https://img.shields.io/badge/Python-3.11-3776AB?style=flat-square&logo=python&logoColor=white) ![Ubuntu 24.04](https://img.shields.io/badge/OS-Ubuntu%2024.04-E95420?style=flat-square&logo=ubuntu&logoColor=white) ![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-00B3E6?style=flat-square&logo=ros) ![Windows 11](https://img.shields.io/badge/OS-Windows%2011-0078D4?style=flat-square&logo=windows&logoColor=white) | [![GitHub 仓库](https://img.shields.io/badge/GitHub-grey?logo=github&style=flat-square)](https://github.com/linker-bot/linkerhand-ros2-sdk) |

# 2. **Version History**
V3.0.1
1. Supports RS485 communication for L6/L10 dexterous hands.
2. Refactored ROS-layer logic to improve performance.

V2.2.4
1. Added support for the G20 Industrial version LinkerHand.
2. Supports RS485 communication protocol for L10.


v2.2.3
1. Supports RS485 communication protocol for O6/L6 (requires an RS485 communication module).

V2.1.9
1. Supports L6/L6P/O6 dexterous hands.

V2.1.8
1. Fixed occasional frame collision issue.
......

# 3. Preparation

## 3.1 System and Hardware Requirements

* Operating System: Ubuntu 20.04

* ROS Version: Noetic

* Python Version: V3.8.10

* Hardware Interface: 5V standard USB port

## 3.2 Download

```python
$ mkdir -p Linker_Hand_SDK_ROS/src    # Create directory
$ cd Linker_Hand_SDK_ROS/src    # Navigate to the directory
$ git clone https://github.com/linker-bot/linkerhand-ros-sdk.git    # Obtain the SDK
```

## 3.3 Install Dependencies and Compilation

```python
$ sudo apt install python3-can
$ cd Linker_Hand_SDK_ROS/src/linker_hand_sdk    # Navigate to the directory
$ pip install -r requirements.txt    # Install required dependencies$ cd Linker_Hand_SDK_ROS # Return to the project root
$ catkin_make    # Compile and build the ROS package
```
## 3.4 Configure ROS Master-Slave Communication

This supports distributed computing and modular development and only takes effect on the current terminal. Ignore this step if not needed. Raspberry Pi devices are configured by default. If you need to configure the ROS master-slave communication, please execute the following commands:

```shell
$ source /opt/ros/noetic/setup.bash
$ export ROS_MASTER_URI=http://<ROS Master IP>:11311
$ export ROS_IP=<本机IP>
$ export ROS_HOSTNAME=<本机IP>
```

# 4. Usage

## 4.1 Modify the setting.yaml Configuration File

Whether running on real hardware or simulation, the configuration parameter file must be modified first.

Currently, the ROS graphical interface control example can only control a single LinkerHand dexterous hand.

```shell
$ cd Linker_Hand_SDK_ROS/src/linker_hand_sdk/linker_hand_sdk_ros/scripts/LinkerHand/config
$ sudo vim setting.yaml    # Edit the configuration file
```

## setting.yaml Description

```yaml
VERSION: 2.0.2 # Refactored core source code, supports motion capture glove speed
LINKER_HAND:  # Hand configuration information
  LEFT_HAND:
    EXISTS: True # Whether the left hand exists. Change to False if it doesn't.
    TOUCH: True  # Whether the pressure sensor is present. Change to False if it's not.
    JOINT: L7 # Left hand joint type: L7 \ L10 \ L20 \ L25
    NAME: # Whether L10 or L20, the joint name list has 20 elements
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
    EXISTS: False # Whether the right hand exists
    TOUCH: True # Whether the pressure sensor is present
    JOINT: L10 # Right hand joint type: L7 \ L10 \ L20 \ L25
    NAME:  # Whether L10 or L20, the joint name list has 20 elements
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
PASSWORD: "12345678" # System administrator password required to activate the CAN communication interface. 
```

## Modify the launch file for parameter configuration
```shell
$ cd /Linker_Hand_SDK_ROS/src/linker_hand_sdk_ros/launch/
$ sudo vim linker_hand.launch    # Launch left OR right single hand (edit config based on comments)
$ sudo vim linker_hand_double.launch    # Launch both left and right hands (edit config based on comments)
# If an error occurs:
ERROR: cannot launch node of type [linker_hand_sdk_ros/linker_hand.py]: Cannot locate node of type [linker_hand.py] in package [linker_hand_sdk_ros]. Make sure file exists in package path and permission is set to executable (chmod +x)
# You need to grant execution permission:
$ sudo chmod a+x src/linkerhand-ros-sdk/linker_hand_sdk_ros/scripts/linker_hand.py
$ roslaunch linker_hand_sdk_ros linker_hand.launch
```
- linker_hand.launch
```html
<?xml version="1.0" encoding="utf-8"?>
<launch>
    <node pkg="linker_hand_sdk_ros" type="linker_hand.py" name="linker_hand_sdk" output="screen" >  <!--  Start the SDK  -->
        <param name="hand_type" type="string" value="right"/> <!--left or right-->
        <param name="hand_joint" type="string" value="L10"/> <!--O6/L6/L7/L10/L20/G20/L21-->
        <param name="touch" type="bool" value="true"/> <!--Is there a pressure sensor-->
    </node>
</launch>
```

### Single USB-to-CAN Control for Dual Hands Note: First, ensure no other CAN devices are connected to the control PC. Connect the USB-to-CAN cables of the same color together. Supports all Linker Hand models with CAN communication.
- Modify linker_hand_double.launch
```html
    <arg name="left_hand_joint" default="L10"/> <!-- left-hand model  O6/L6/L7/L10/L20/G20/L21-->
    <arg name="right_hand_joint" default="L10"/> <!-- right-hand model  O6/L6/L7/L10/L20/G20/L21-->
    <arg name="left_touch" default="true"/> <!-- Left hand pressure sensor true or false-->
    <arg name="right_touch" default="true"/> <!-- Right hand pressure sensor true or false-->
    <arg name="left_can" default="can0"/> <!-- Left-hand USB to CAN converter serial number can0-->
    <arg name="right_can" default="can0"/> <!-- Right-hand USB to CAN converter serial number can0-->
```

### Dual USB to CAN control with two-hand operation Note: First, ensure that no other CAN devices are connected to the control computer. Insert the left USB to CAN adapter as can0, and then insert the right USB to CAN adapter as can1. Supports CAN communication for all Linker Hand models
- Modify `linker_hand_double.launch`
```html
    <arg name="left_hand_joint" default="L10"/> <!-- left-hand model O6/L6/L7/L10/L20/G20/L21-->
    <arg name="right_hand_joint" default="L10"/> <!-- right-hand model O6/L6/L7/L10/L20/G20/L21-->
    <arg name="left_touch" default="true"/> <!-- Left hand pressure sensor true or false-->
    <arg name="right_touch" default="true"/> <!-- Right hand pressure sensor true or false-->
    <arg name="left_can" default="can0"/> <!-- Left-hand USB to CAN converter serial number can0-->
    <arg name="right_can" default="can1"/> <!-- Right-hand USB to CAN converter serial number can0-->
```
## 4.2 LinkerHand Dexterous Hand and PC Hardware Connection

### 4.2.1 Insert the LinkerHand's USB-to-CAN adapter into the Ubuntu device. The blue light should illuminate.

![](https://lkaeimso7m.feishu.cn/space/api/box/stream/download/asynccode/?code=YzkzNzI4YmE1MzdmMzMzOGFhNmUyMDQ4YjViMDk0N2JfZGxnTHZMaU9Ec2dqdkZKOERBeTcxU3BScDN1QTdPcXJfVG9rZW46RkNZYWIxNWllb0NlMW94SkR5aGNhcEs0bm9oXzE3NDM1ODU1MDk6MTc0MzU4OTEwOV9WNA)

**Light Blinking**: A blinking blue light indicates successful communication.

**Communication Mode**: How to distinguish between CAN and RS485 when connecting via the CAN module? The CAN and RS485 interfaces are completely different, and the connection methods are also different.

## 4.3 Starting the SDK

Start the LinkerHand L10, L20 dexterous hand SDK. Upon successful launch, there will be prompt information regarding the SDK version, CAN interface status, dexterous hand configuration, and current joint speeds.&#x20;

```shell
# Open the CAN port
$ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 # USB-to-CAN device blue light stays solid. This step can be skipped on Ubuntu systems after modifying setting.ymal as required.
$ cd ~/Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch left OR right single hand
or
$ roslaunch linker_hand_sdk_ros linker_hand_double.launch # Launch both left and right hands
# If an error occurs:
ERROR: cannot launch node of type [linker_hand_sdk_ros/linker_hand.py]: Cannot locate node of type [linker_hand.py] in package [linker_hand_sdk_ros]. Make sure file exists in package path and permission is set to executable (chmod +x)
# You need to grant execution permission:
$ sudo chmod a+x src/linker_hand_sdk/linker_hand_sdk_ros/scripts/linker_hand.py
$ roslaunch linker_hand_sdk_ros linker_hand.launch
```

### 4.4 RS485 Protocol Switching (Currently supports O6/L6; refer to the MODBUS RS485 protocol document for other models)

Edit the configuration file at [scripts/LinkerHand/config/setting.yaml](https://github.com/linker-bot/linkerhand-ros-sdk/blob/main/linker_hand_sdk_ros/scripts/LinkerHand/config/setting.yaml) and modify the parameters according to the comments within the file, setting MODBUS: "/dev/ttyUSB0", and setting the modbus parameter in [linker_hand.launch.py](https://github.com/linker-bot/linkerhand-ros-sdk/blob/main/linker_hand_sdk_ros/launch/linker_hand.launch) to "/dev/ttyUSB0". The USB-RS485 converter typically appears as /dev/ttyUSB* or /dev/ttyACM* on Ubuntu. modbus: "None" or "/dev/ttyUSB0" Note: The modbus parameter is a string type; if it is not "None", the can parameter will be ignored.
```bash
# Ensure requirements.txt dependencies are installed
# Install system-level drivers
$ pip install minimalmodbus
$ pip install pyserial
$ pip install pymodbus==3.5.1
# Check the USB-RS485 port number
$ ls /dev
# You should see a port similar to ttyUSB0. Grant execution permission:
$ sudo chmod 777 /dev/ttyUSB0
```

## Position to Finger Joint Mapping Table
  ```bash
  $ rostopic echo /cb_left_hand_control_cmd
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

  L6/O6: ["Thumb Bend", "Thumb Yaw", "Index Finger Bend", "Middle Finger Bend", "Ring Finger Bend", "Little Finger Bend"]
  
  L7:  ["Thumb Bend", "Thumb Yaw", "Index Finger Bend", "Middle Finger Bend", "Ring Finger Bend", "Little Finger Bend", "Thumb Roll"]

  L10: ["Thumb Proximal", "Thumb Side Bend", "Index Proximal", "Middle Proximal", "Ring Proximal", "Little Proximal", "Index Side Bend", "Ring Side Bend", "Little Side Bend", "Thumb Roll"]

  L20: ["Thumb Proximal", "Index Proximal", "Middle Proximal", "Ring Proximal", "Little Proximal", "Thumb Side Bend", "Index Side Bend", "Middle Side Bend", "Ring Side Bend", "Little Side Bend", "Thumb Yaw", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Tip", "Index Tip", "Middle Tip", "Ring Tip", "Little Tip"]

  G20 (Industrial): ["Thumb Proximal", "Index Proximal", "Middle Proximal", "Ring Proximal", "Little Proximal", "Thumb Side Bend", "Index Side Bend", "Middle Side Bend", "Ring Side Bend", "Little Side Bend", "Thumb Yaw", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Tip", "Index Tip", "Middle Tip", "Ring Tip", "Little Tip"]

  L21: ["Thumb Proximal", "Index Proximal", "Middle Proximal", "Ring Proximal", "Little Proximal", "Thumb Side Bend", "Index Side Bend", "Middle Side Bend", "Ring Side Bend", "Little Side Bend", "Thumb Roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Medial", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Distal", "Index Distal", "Middle Distal", "Ring Distal", "Little Distal"]

  L25: ["Thumb Proximal", "Index Proximal", "Middle Proximal", "Ring Proximal", "Little Proximal", "Thumb Side Bend", "Index Side Bend", "Middle Side Bend", "Ring Side Bend", "Little Side Bend", "Thumb Roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb Medial", "Index Medial", "Middle Medial", "Ring Medial", "Little Medial", "Thumb Distal", "Index Distal", "Middle Distal", "Ring Distal", "Little Distal"]

# 5. **Package Introduction**

## 5.1 linker_hand_sdk_ros

Controls the dexterous hand joint angles and retrieves various status information from the dexterous hand.


## 5.2 examples

Contains usage examples for various products.

## 5.3 doc

Directory for documentation attachments.
