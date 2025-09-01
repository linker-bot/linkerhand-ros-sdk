# 1. **Overview**

Clever Hands, Create Everything.

LinkerHand Dexterous Hand ROS SDK is a software tool developed by CHIUS INC to drive its series of dexterous hand products and provide functional examples. It supports various devices (such as laptops, desktops, Raspberry Pi, Jetson, etc.) and mainly serves fields like humanoid robotics, industrial automation, and scientific research institutions. It is suitable for scenarios such as humanoid robots, flexible automatic product lines, embodied large model training, and data collection.

**Warning**

1. Please stay away from the dexterous hand's range of motion to avoid personal injury or equipment damage.

2. Be sure to conduct a safety assessment before performing any actions to prevent collisions.

3. Please take good care of the dexterous hand.

# 2. **Version Information**
V2.1.9
1. Support L6\L6P\O6 dexterous hands
V2.1.8
1. Fix occasional frame collision issues

V1.3.4

1. The waveform graph display has been changed from single-hand to configurable single/dual-hand display, controlled by modifying the configuration file to determine whether pressure sensing is present.

2. Resolved data alignment errors in the proximity sensing waveform graph.

3. Modify the CAN port closure logic to prevent sensor data from being unreadable on the robotic hand after the second system boot..

V1.3.3

1. GUI now includes a waveform graph for pressure sensors.

2. L10 now supports setting speed and torque.

V1.3.2

1. Added support for the T24 version of the dexterous hand.

V1.3.1

1. Added acquisition of LinkerHand dexterous hand state values (radians and range) in examples

2. Added PyBullet simulation environment

3. Added GUI control interface

# 3. Preparation

## 3.1 System and hardware requirements

* Operating System: Ubuntu20.04

* ROS Version:Noetic

* Python Version: V3.8.10

* Hardware Interface: 5V standard USB interface

## 3.2 Download

```python
$ mkdir -p Linker_Hand_SDK_ROS/src    #Make directory
$ cd Linker_Hand_SDK_ROS/src    #Navigate to the directory 
$ git clone https://github.com/linker-bot/linkerhand-ros-sdk.git   #Get SDK
```

## 3.3 Install dependencies and compile

```python
$ sudo apt install python3-can
$ cd Linker_Hand_SDK_ROS/src/linker_hand_sdk    #Navigate to the directory 
$ pip install -r requirements.txt    #Install required dependencies
$ catkin_make    #Compile and build the ROS package
```

## 3.4 Configuring ROS Master-Slave Communication

Supports distributed computationand modularization development, effective only at this terminal; ignore if not needed. Raspberry Pi devices are pre-configured by default.

```shell
$ source /opt/ros/noetic/setup.bash
$ export ROS_MASTER_URI=http://<ROS Master IP>:11311
$ export ROS_IP=<host IP>
$ export ROS_HOSTNAME=<host IP>
```

# 4. Usage

## 4.1 Modify the setting.yaml configuration file

Whether operating on actual hardware or in a simulation, the configuration parameters must be altered beforehand.

Currently, the graphical user interface control example for ROS development is capable of independently manipulating only one LinkerHand robotic arm at a time.

```python
$ cd Linker_Hand_SDK_ROS/src/linker_hand_sdk/linker_hand_sdk_ros/config
$ sudo vim setting.yaml    #Edit the configuration file
```

Description of setting.yaml

```yaml
VERSION: 1.3.5 # Version Number, L7 O7 Supported
LINKER_HAND:  # Hand configuration information
  LEFT_HAND:
    EXISTS: True # Check if the left hand exists. If it does not, set the value to False.
    TOUCH: True  # Check if the pressure sensor exists. If it does not, set the value to False.
    JOINT: L7 # Number of joints in the left hand L7 \ L10 \ L20 \ L25
    NAME: # Regardless of l10 or l20, joint name always has 20 entries
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
    EXISTS: False # Check if the right hand exists.
    TOUCH: True # Check if the pressure sensor exists.
    JOINT: L10 # Number of joints in the right hand L7 \ L10 \ L20 \ L25
    NAME:  # Regardless of l10 or l20, joint name always has 20 entries
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
PASSWORD: "12345678" # Due to communication with CAN, the system administrator password is required to activate the communication interface.
```

## 4.2 Connect the LinkerHand Dexterous Hand hardware to your PC

### 4.2.1 Insert the USB-to-CAN device interface of the LinkerHand dexterous hand into the Ubuntu device; the blue light will turn on.

![](https://lkaeimso7m.feishu.cn/space/api/box/stream/download/asynccode/?code=ODI5Zjk4NzhiN2U4ZWNhNjIzMTdmOTE1ZWI5ODRlNjFfbjhEaEQ2dlVBN2J1WlpaWFFYSUdmMmtzd2FteldWZDFfVG9rZW46UGVWQmI3b2gwb1ZJbEN4N3ZuQWN5c0g4bmdjXzE3NDM1ODU1NTU6MTc0MzU4OTE1NV9WNA)

Light indicator: Flashing blue signifies a successful connection.

## 4.3 Launch SDK

Launch the LinkerHand L10, L20 dexterous hand SDK. Upon successful startup, there will be prompts for SDK version, CAN interface status, dexterous hand configuration information, and current joint speed of the dexterous hand.

```python
# Enable CAN port
$ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 #USB-to-CAN device with blue light constantly on
$ cd ~/Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

Launch the LinkerHand O7 dexterous hand SDK.

```python
# Enable CAN port
$ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 #USB-to-CAN device with blue light constantly on
$ cd ~/Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand_l7.launch # Launch L07 dexterous hand
```

Launch the LinkerHand L25 dexterous hand SDK.

```python
# Enable CAN port
$ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 #USB-to-CAN device with blue light constantly on
$ cd ~/Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand_l25.launch # Launch L07 dexterous hand
```

# 5. **ROS Package Overview**

## 5.1 linker\_hand\_sdk\_ros

Actuate joint angles of the LinkerHand and retrieve its real-time state values (angle in radians, angular range).

## 5.2 range\_to\_arc

Retrieve radian measurements from L10 and L20.

Get and send the radians for L10 or L20  through the topic:/cb_left_hand_state_arc and /cb_right_hand_state_arc 

Get the position status of the LinkerHand in radians through the topic topic:/cb_left_hand_control_cmd_arc and /cb_right_hand_control_cmd_arc.  Publish the position in radians to control the finger movement of the LinkerHand.

## 5.3 examples

Includes application examples for each product.

## 5.4 doc

Appendix: Document Attachments

# 6. **ROS Development Examples**

## 6.1 PyBullet Simulation

Supported LinkerHand products: L10、L20、L25

1. Open a new terminal and launch ROS

```css
$ roscore
```

* Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

* Open a new terminal and launch PyBullet Simulation

```python
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun linker_hand_pybullet linker_hand_pybullet.py _hand_type:=L20
```

**Parameter Description**

The value of ‘\_hand\_type‘ can be selected based on the product model, for example: \_hand\_type:=L20

**Output Result Example**

None.

## 6.2 **Graphical User Interface Control**

Supported LinkerHand products: L10、L20

Graphical user interface control allows the independent movement of each joint of the LinkerHand dexterous hand L10 and L20 via sliders. Buttons can also be added to record the current values of all sliders and save the current movement state of each joint of the LinkerHand dexterous hand. Functional buttons can be used to replay the action.

Use gui\_control to control the LinkerHand Dexterous Hand: To operate the LinkerHand Dexterous Hand via the gui\_control interface, you need to launch the linker\_hand\_sdk\_ros package and communicate via topics.

1. Open a new terminal and launch ROS

```python
$ roscore
```

* Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

* Open a new terminal and launch the graphical user interface control.

```python
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun gui_control gui_control.py
```

After opening, a UI interface appears. Users can control the joints of the LinkerHand dexterous hand using sliding bars. They can also save the current sliding bar data by clicking the add button on the right, facilitating the replay of the settings in the future.

**Parameter Description**

None.

**Output Result Example**

None.

## 6.3 Get Robot Status Information

### 6.3.1 **Get Current Status**

Supported LinkerHand products: L10、L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

* Open a new terminal and get the current status

  1. For L20, open a new terminal to get the current status.

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ rosrun L20_get_linker_hand_state L20_get_linker_hand_state.py _loop:=True
```

**Parameter Description**&#x20;

Status values include: range value and radian value.

The “\_loop“ parameter must be specified. If set to True, the terminal will continuously display the current status values of the LinkerHand dexterous hand. If set to False, the terminal will display the status once. For example: \_loop:=True .

**Output Result Example**

稍后更新

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ rosrun L10_get_linker_hand_state L10_get_linker_hand_state.py _loop:=True
```

**Parameter Description&#x20;**

Status values include: range value and radian value.

The “\_loop“ parameter must be specified. If set to True, the terminal will continuously display the current status values of the LinkerHand dexterous hand. If set to False, the terminal will display the status once. For example: \_loop:=True.

**Output Result Example**

```bash
header:
  seq: 83
  stamp:
    secs: 1743409242
    nsecs: 193927526
  frame_id: ''
name: []
position: [1.03, -1.57, 1.3, 1.3, 1.3, 1.3, 0.26, -0.26, -0.26, 1.57]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### 6.3.2 **Get force sensor data**

Supported LinkerHand products: L10、L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

* Open a new terminal and get force sensor data

```python
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun get_linker_hand_force get_linker_hand_force.py _loop:=False
```

**Parameter Description&#x20;**

The “\_loop“ parameter must be specified. If set to True, the terminal will continuously display the current status values of the LinkerHand dexterous hand. If set to False, the terminal will display the status once. For example: \_loop:=True.

**Output Result Example**

* Right hand five-finger normal force readings: \[0.0, 0.0, 0.0, 0.0, 0.0], with a range of 0 to 255, where greater pressure results in higher values.

* Right hand five-finger tangential force readings: \[0.0, 0.0, 0.0, 0.0, 0.0], with a range of 0 to 255, where greater pressure results in higher values.

* Right hand five-finger tangential force direction readings: \[255.0, 255.0, 255.0, 255.0, 255.0], with a range of 255 to 0, where greater pressure results in smaller values.

* Right hand five-finger proximity sensing readings: \[0.0, 0.0, 0.0, 0.0, 0.0], with a range of 0 to 255, where greater pressure results in higher values.

### 6.3.3 **Get the Current Velocity**

Supported LinkerHand products: L10、L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

* Open a new terminal and get force sensor data

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ rosrun get_linker_hand_speed get_linker_hand_speed.py _loop:=False
```

**Parameter Description**

The “\_loop“ parameter must be specified. If set to True, the terminal will continuously display the current status values of the LinkerHand dexterous hand. If set to False, the terminal will display the status once. For example: \_loop:=True.

**Output Result Example**

The velocity of the right hand’s five fingers is: \[180, 250, 250, 250, 250], in the order of thumb, index finger, middle finger, ring finger, and little finger.

### 6.3.4 **Get the Current**

Supported LinkerHand products: L10、L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

* Open a new terminal and get the current

```python
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun get_linker_hand_current get_linker_hand_current.py _loop:=False
```

**Parameter Description**

The “\_loop“ parameter must be specified. If set to True, the terminal will continuously display the current status values of the LinkerHand dexterous hand. If set to False, the terminal will display the status once. For example: \_loop:=True.

**Output Result Example**

The current of the right hand’s five fingers is: \[180, 250, 250, 250, 250], in the order of thumb, index finger, middle finger, ring finger, and little finger.

### 6.3.5 **Get Error Code**

```python
rostopic pub /cb_hand_setting_cmd std_msgs/String '{data: "{\"setting_cmd\":\"get_faults\",\"params\":{\"hand_type\":\"left\"}}"}'
```

**Parameter Description**

setting\_cmd : Command Parameter

get\_faults : Command Type String

**Output Result Example**

Current of the right hand’s five fingers is: \[0, 1, 0, 0, 0], where 0 represents normal and 1 represents a fault.

## 6.4 Settings

### 6.4.1 Set Speed

Supported LinkerHand products: L10、L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

* Open a new terminal and set speed

```python
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun set_linker_hand_speed set_linker_hand_speed.py _hand_type:=left _speed:=[180,250,250,250,250] # L7为7个值，其他为5个值
```

**Parameter Description**

L10, L20: Consistent speed of five fingers

speed:=\[180,250,250,250,250]

L7：Speed of seven motors

speed:=\[180,250,250,250,250,250,250]&#x20;

**Output Result Example**

speed:\[180,250,250,250,250]

### 6.4.2 Set Current

Supported LinkerHand products: L10、L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

* Open a new terminal and set the current

```python
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun set_linker_hand_current set_linker_hand_current.py _hand_type:=left _current:=42 #暂不支持L7
```

**Parameter Description**

Parameters: hand\_type: left | right (left or right hand) current:0\~255 (set maximum current value)

**Output Result Example**

current:\[180,250,250,250,250]

### 6.4.3 Set Torque

Supported LinkerHand products: L10、L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

* Open a new terminal and set torque

```python
$ cd Linker_Hand_SDK_ROS
$ source ./devel/setup.bash
$ rosrun set_linker_hand_torque set_linker_hand_torque.py _hand_type:=left _torque:=[180,250,250,250,250] # L7为7个值，其他为5个值
```

**Parameter Description**

Parameters: hand\_type: left | right (left or right hand) torque:=\[180,250,250,250,250]  0\~255. L7 represents the maximum torque for 7 motors, while the other values represent the maximum torque for 5 fingers.&#x20;

**Output Result Example**

torque:\[180,250,250,250,250]

### 6.4.4 **Set to disabled mode**

Supported LinkerHand products: L25

Disables the motor of the dexterous hand, allowing free movement of all joints.

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L25 dexterous hand
```

* Open a new terminal and set to disabled mode

```python
$ Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/L25
$ python set_disability.py
```

**Parameter Description**

None

**Output Result Example**

None

### 6.4.5 **Set to enabled mode**

Supported LinkerHand products: L25

Enable the dexterous hand motor, which can then be controlled by the control program.

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L25 dexterous hand
```

* Open a new terminal and set to enabled mode

```python
$ Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/L25
$ python set_enable.py
```

**Parameter Description**

None

**Output Result Example**

None

### 6.4.6 **Set to remote operation mode**

Supported LinkerHand products: L25

If you possess multiple L25 dexterous hands of the same version, this example demonstrates how to have a disabled hand control an enabled one.

First, start the LinkerHand SDK ROS. Below is the configuration method for the controlled L25 dexterous hand, using the right hand as an example. Ensure that both Ubuntu machines are on the same network and that the master and slave configurations are set up, allowing both machines to communicate via ROS simultaneously. For further guidance, please refer to the [Documentation - ROS Wiki](https://wiki.ros.org/).

**Control Party A Dexterous Hand Configuration**

1. Open a new terminal and launch ROS

```css
$ roscore
```

* Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L25 dexterous hand
```

* Open a new terminal and start executing remote control

```python
$ Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/L25
$ python set_remote_control.py
```

**Controlled Party B Dexterous Hand Configuration**

1. Open a new terminal and launch ROS

```python
# Open a new terminal and launch ROS
$ roscore
```

* Open a new terminal

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch
```

At this time, manually dragging the disabled L25 dexterous hand of Machine A can control the enabled L25 dexterous hand of Machine B.

## 6.5 **Application Demonstration**

### 6.5.1 **Rock-Paper-Scissors Game**

Supported LinkerHand products: L10、L20

Note: Requires an RGB camera

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L10 or L20 dexterous hand
```

* Open a new terminal, start Rock-Paper-Scissors game

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ rosrun finger_guessing finger_guessing.py
```

**Parameter Description**

None

**Output Result Example**

None

### 6.5.2 **Pinch Operation**

Supported LinkerHand products: L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L20 dexterous hand
```

* Open a new terminal , start Pinch Demonstration

```python
python ./<Your path>/lipcontroller.py
```

If the terminal prints ‘**Start Demonstration**’, it indicates normal operation. At this point, if the hand settings are correct, it should begin performing a pinch motion using the index finger and middle finger. The motion will stop when an object is pinched, and after removing the object, it will continue attempting to pinch until it successfully grabs the item or reaches its motion limit.

**lipcontroller.py&#x20;**&#x69;s a demonstration demo developed based on version 7 of product O7. When used in demonstrations of other versions, you need to adjust the closing posture of the thumb and index finger. Otherwise, the action of ‘**pinching together with the index finger and thumb’** cannot be achieved.

**Parameter Description**

None

**Output Result Example**

None

## 6.6 **Imitation Learning**

### 6.6.1 **Imitation Learning Training**

The hardware used in this example is the LinkerRobot humanoid robot, but other robotic arms or robots can also be used for imitation learning training, as long as the corresponding data topics are modified. For detailed instructions, please refer to the [human-dex project README.md](https://github.com/linkerbotai/human-dex)&#x20;

1. Configure environment

```css
cd human-dex
conda create -n human-dex python=3.8.10
conda activate human-dex
pip install torchvision
pip install torch
pip install -r requirements.txt
```

* Installation

```css
mkdir -p your_ws/src
cd your_ws/src
git clone https://github.com/linker-bot/human-dex.git
cd ..
catkin_make
source ./devel/setup.bash
```

* Run

```solidity
# Data Collection
 roslaunch record_hdf5 record_hdf5.launch
# Open a new terminal to send the collection command
rostopic pub /record_hdf5 std_msgs/String "data: '{\"method\":\"start\",\"type\":\"humanplus\"}'"
```

* Train

```solidity
cd humanplus/scripts/utils/HIT
python3 imitate_episodes_h1_train.py --task_name data_cb_grasp --ckpt_dir cb_grasp/ --policy_class HIT --chunk_size 50 --hidden_dim 512 --batch_size 48 --dim_feedforward 512 --lr 1e-5 --seed 0 --num_steps 100000 --eval_every 1000 --validate_every 1000 --save_every 1000 --no_encoder --backbone resnet18 --same_backbones --use_pos_embd_image 1 --use_pos_embd_action 1 --dec_layers 6 --gpu_id 0 --feature_loss_weight 0.005 --use_mask --data_aug
```

* Reproduction/Evaluation

```css
cd humanplus/scripts
python3 cb.py
```

**Parameter Description**

None

**Output Result Example**

None

### 6.6.2 **Unidexgrasp Grasping Algorithm**

The original Unidexgrasp algorithm uses the shadowhand. Below is the relevant code for developing the Unidexgrasp algorithm on the linkerhand. For the detailed process, refer to the [linker\_unidexgrasp project](https://github.com/linkerbotai/linker_unidexgrasp).

**Grasping Pose Generation Section**

The grasping pose module maps the shadowhand pose output by the model to the linkerHand L20 pose, facilitating its use in subsequent development.

1. Configure environment

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

* Train&#x20;

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

* Verification

```bash
python ./network/eval.py  --config-name eval_config \
                          --exp-dir=./eval
```

* Mapping, result visualization

```bash
python ./tests/visualize_result_l20_shadow.py --exp_dir 'eval' --num 3
```

* Save the results for subsequent reinforcement learning algorithm development use.

```bash
python ./tests/data_for_RL.py
```

# 7. Python Development Example

### 7.1 **Make an OK gesture&#x20;**

Supported LinkerHand products: L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L20 dexterous hand
```

* Open a new terminal and make an OK gesture

```python
python ./<Your path>/gesture-Show-OK.py
```

After starting, the terminal will print “Testing in progress”, and at the same time, the hand will begin to make an OK gesture, bending the middle, ring, and little fingers while extending them.

**Parameter Description**

None

**Output Result Example**

None

### 7.2&#x20;**&#x20;Hand performs index finger rotation movement**

Supported LinkerHand products: L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L20 dexterous hand
```

* Open a new terminal and start the index finger rotation movement

```python
python ./<Your path>/gesture-Show-Surround-Index-Finger.py
```

After starting, the terminal will print "Testing in progress". At this time, the hand will start to clench into a fist and extend the index finger, which will continuously rotate.

**Parameter Description**

None

**Output Result Example**

None

### 7.3&#x20;**&#x20;Hand wave motion**

Supported LinkerHand products: L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L20 dexterous hand
```

* Open a new terminal and start the wave motion

```python
python ./<Your path>/gesture-Show-Wave.p
```

After starting, the terminal will print "Testing in progress". At this time, the thumb extends outward and remains still, while the other four fingers begin the wave motion.

**Parameter Description**

None

**Output Result Example**

None

### 7.4 **Hand Performs a Set of Complex Movements**

Supported LinkerHand products: L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L20 dexterous hand
```

* Open a new terminal and start a set of complex movements

```python
python ./<Your path>/gesture-Show-Ye.py
```

After starting, the terminal will print "Testing in progress". At this time, the hand will begin to perform a set of complex movements to demonstrate the flexibility of the hand.

This example is a demonstration demo developed based on version 7 of product O7. When used in demonstrations of other versions, you need to adjust the closing posture of the thumb and index finger. Otherwise, the action of ‘**pinching together with the index finger and thumb’** cannot be achieved.

**Parameter Description**

None

**Output Result Example**

None

### 7.5&#x20;**&#x20;Manual Loop Grasping Action**

Supported LinkerHand products: L20

1. Open a new terminal and launch ROS

```css
$ roscore
```

* 2\. Open a new terminal and launch Linker Hand ROS SDK

```python
$ cd Linker_Hand_SDK_ROS/
$ source ./devel/setup.bash
$ roslaunch linker_hand_sdk_ros linker_hand.launch # Launch L20 dexterous hand
```

* Open a new terminal and start the loop grasp action

```python
$ cd Linker_Hand_SDK_ROS/src/linker_hand_sdk/examples/gesture-show
$ python gesture-Loop.py 
```

**Parameter Description**

None

**Output Result Example**

None

### 7.6 Finger Dance

Supported LinkerHand products: L25

**Parameter Description**

None

**Output Result Example**

None

# 8. Related GitHub Resources

Human-Dex：https://github.com/linkerbotai/human-dex

Linker\_UniDexGrasp：https://github.com/linkerbotai/linker\_unidexgrasp

LinkerHand-Python-SDK：https://github.com/linkerbotai/linker\_hand\_python\_sdk

linker\_serl：https://github.com/linkerbotai/linker\_serl

# 9. FAQ

## 9.1 Installing the CAN Module Driver

```python
pip install python-can
```

## 9.2 How to avoid entering the password repeatedly?

Method One

```python
$ sudo visudo
# Add the following configuration
you_name ALL=(ALL) NOPASSWD: /sbin/ip
you_name ALL=(ALL) NOPASSWD: /usr/sbin/ip link set can0 up type $ $ can bitrate 1000000
# Save and exit
```

Method Two

Modify the password in the setting.yaml configuration file, default PASSWORD: "12345678"


## Topic Document
[Linker Hand Topic Document](doc/Topic-Reference.md)
