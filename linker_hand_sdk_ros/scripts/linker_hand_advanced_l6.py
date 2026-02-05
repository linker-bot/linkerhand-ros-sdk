#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rospy, signal, rospkg, sys, os, math, time, threading, json,itertools
import argparse
import numpy as np
from std_msgs.msg import String, Header, Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState,PointCloud2, PointField
from common.init_position import CONFIG
from LinkerHand.utils.mapping import *
from LinkerHand.linker_hand_api import LinkerHandApi
from LinkerHand.utils.init_linker_hand import InitLinkerHand
from LinkerHand.utils.load_write_yaml import LoadWriteYaml
from LinkerHand.utils.color_msg import ColorMsg
from LinkerHand.utils.open_can import OpenCan

class LinkerHandAdvancedL6:
    def __init__(self,hand_type="right", hand_joint="L6", is_touch="true", can="can0", moudbus="None"):
        self.hand_type = hand_type
        self.hand_joint = hand_joint
        self.is_touch = is_touch
        self.can = can
        self.modbus = moudbus
        self.is_lock = False
        self.sdk_v = 0
        self.last_position_cmd = None # 接收到的最后一条控制指令
        self.last_velocity_cmd = None # 接收到的最后一条控制指令
        self.last_hand_state = { # 最后获取到的状态数值
            "state": [-1] * 5,
            "vel": [-1] * 5
        }
        self.open_can = OpenCan()
        self.open_can.open_can(self.can)
        self.last_touch_force = [-1] * 5
        self.matrix_dic = {
            "stamp":{
                "secs": 0,
                "nsecs": 0,
            },
            "thumb_matrix":[[-1] * 6 for _ in range(12)],
            "index_matrix":[[-1] * 6 for _ in range(12)],
            "middle_matrix":[[-1] * 6 for _ in range(12)],
            "ring_matrix":[[-1] * 6 for _ in range(12)],
            "little_matrix":[[-1] * 6 for _ in range(12)]
        }
        # 压感矩阵合值，单位g 克
        self.matrix_mass_dic = {
            "stamp":{
                "secs": 0,
                "nsecs": 0,
            },
            "thumb_mass":[-1],
            "index_mass":[-1],
            "middle_mass":[-1],
            "ring_mass":[-1],
            "little_mass":[-1]
        }
        self.last_hand_info = {
            "version": [-1], # Dexterous hand version number
            "hand_joint": self.hand_joint, # Dexterous hand joint type
            "speed": [-1] * 10, # Current speed threshold of the dexterous hand
            "current": [-1] * 10, # Current of the dexterous hand
            "fault": [-1] * 10, # Current fault of the dexterous hand
            "motor_temperature": [-1] * 10, # Current motor temperature of the dexterous hand
            "torque": [-1] * 10, # Current torque of the dexterous hand
            "is_touch":self.is_touch,
            "touch_type": -1,
            "finger_order": None # Finger motor order
        }
        self._init_hand()
        self.thread_get_state = threading.Thread(target=self.get_state)
        self.thread_get_state.daemon = True
        self.thread_get_state.start()


    def _init_hand(self):
        self.api = LinkerHandApi(hand_type=self.hand_type, hand_joint=self.hand_joint,can=self.can,modbus=self.modbus)
        # 获取Linker Hand版本号
        self.embedded_version = self.api.get_embedded_version()
        # 获取指尖压感类型
        self.touch_type = self.api.get_touch_type()
        self.hand_cmd_sub = rospy.Subscriber(f"/cb_{self.hand_type}_hand_control_cmd", JointState, self.hand_cmd_cb, queue_size=10)
        self.hand_state_pub = rospy.Publisher(f'/cb_{self.hand_type}_hand_state', JointState, queue_size=10)
        if self.is_touch == True:
            if self.touch_type > 1:
                self.matrix_touch_pub = rospy.Publisher(f"/cb_{self.hand_type}_hand_matrix_touch" ,String, queue_size=10)
                self.matrix_touch_pub_pc = rospy.Publisher(f"/cb_{self.hand_type}_hand_matrix_touch_pc2" ,PointCloud2, queue_size=10)
                self.matrix_touch_pub_mass = rospy.Publisher(f"/cb_{self.hand_type}_hand_matrix_touch_mass" ,String, queue_size=10)
                ColorMsg(msg=f"Linker Hand {self.hand_type} {self.hand_joint} It features a matrix pressure sensor and has been enabled in the configuration",color="green")
            elif self.touch_type != -1:
                self.touch_pub = rospy.Publisher(f"/cb_{self.hand_type}_hand_touch", Float32MultiArray, queue_size=10)
                ColorMsg(msg=f"Linker Hand {self.hand_type} {self.hand_joint} It features a matrix pressure sensor and has been enabled in the configuration",color="green")
        else:
            ColorMsg(msg=f"Linker Hand {self.hand_type} {self.hand_joint} Not opened or equipped with a pressure sensor",color="yellow")
        # 1. 初始化默认值
        pose = None
        torque = [255, 255, 255, 255, 255]
        speed = [255, 255, 255, 255, 255]
        # 2. 查找配置（将手关节名称转换为大写，以匹配字典键）
        joint_key = self.hand_joint.upper()
        # 3. 处理 O6/L6/L6P 的情况 (它们共享相同的配置)
        if joint_key in ("O6", "L6", "L6P"):
            joint_key = "O6" # 将它们都映射到 "O6" 的配置上
        # 4. 获取配置并更新参数
        config_data = CONFIG.get(joint_key)
        if config_data:
            # 仅当配置中提供了 pose/torque/speed 时才更新
            pose = config_data.get("pose", pose)
            torque = config_data.get("torque", torque)
            speed = config_data.get("speed", speed)
        if pose is not None:
            self.api.set_speed(speed=speed)
            self.api.set_torque(torque=torque)
            self.api.finger_move(pose=pose)

    def hand_cmd_cb(self, msg):
        pose = list(msg.position)
        vel = list(msg.velocity)
        if len(pose) == 0:
            return
        else:
            self.last_position_cmd = pose
            self.last_velocity_cmd = vel
        time.sleep(0.03)

    def joint_state_msg(self, pose,vel=[]):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ["thumb_cmc_pitch","thumb_cmc_yaw","index_mcp_pitch","middle_mcp_pitch","pinky_mcp_pitch","ring_mcp_pitch"]
        joint_state.position = pose
        if len(vel) > 1:
            joint_state.velocity = vel
        return joint_state
    
    def get_state(self):
        while True:
            """如果有命令，则执行命令动作"""
            if self.last_position_cmd != None:
                self.api.finger_move(pose=self.last_position_cmd)
                time.sleep(0.002)
                # 重置命令
                self.last_position_cmd = None
                self.last_velocity_cmd = None
            """获取手当前状态"""
            if self.hand_state_pub.get_num_connections() > 0:
                state = self.api.get_state()
                vel = self.api.get_joint_speed()
                self.last_hand_state['state'] = state
                self.last_hand_state['vel'] = vel
                time.sleep(0.003)
            if self.is_touch == True and self.touch_type > 1 and (self.matrix_touch_pub.get_num_connections() > 0 or self.matrix_touch_pub_pc.get_num_connections() > 0 or self.matrix_touch_pub_mass.get_num_connections() > 0):
                """矩阵式压力传感器"""
                self.matrix_dic["thumb_matrix"] = self.api.get_thumb_matrix_touch(sleep_time=0.003).tolist()
                self.matrix_dic["index_matrix"] = self.api.get_index_matrix_touch(sleep_time=0.003).tolist()
                self.matrix_dic["middle_matrix"] = self.api.get_middle_matrix_touch(sleep_time=0.003).tolist()
                self.matrix_dic["ring_matrix"] = self.api.get_ring_matrix_touch(sleep_time=0.003).tolist()
                self.matrix_dic["little_matrix"] = self.api.get_little_matrix_touch(sleep_time=0.003).tolist()
            state_msg = self.joint_state_msg(pose=self.last_hand_state['state'], vel=self.last_hand_state['vel'])
            self.hand_state_pub.publish(state_msg)
            if self.is_touch == True:
                # 发布矩阵压感原始值
                self.pub_matrix_dic()
                # 发布矩阵压感点云值
                self.pub_matrix_point_cloud()
                # 发布矩阵压感合值
                self.pub_matrix_mass(dic=self.matrix_dic)
            time.sleep(0.003)
            


    def pub_matrix_mass(self, dic):
        """发布矩阵数据合值 单位g 克 JSON格式"""
        msg = String()
        # 尝试获取当前的 ROS 时间
        try:
            current_time = rospy.Time.now()
            # 提取 secs 和 nsecs
            t_secs = current_time.secs
            t_nsecs = current_time.nsecs
        except rospy.ROSInitException:
            # 如果 ROS 时间系统尚未启动 (例如，没有 roscore)，使用系统时间作为备用
            # 这种情况下，时间可能不够精确或与 ROS bag 时间不同步
            t_secs = int(time.time())
            t_nsecs = int((time.time() - t_secs) * 1e9)
            rospy.logwarn("ROS Time not available, using system time.")
        self.matrix_mass_dic["stamp"]["secs"] = t_secs
        self.matrix_mass_dic["stamp"]["nsecs"] = t_nsecs
        self.matrix_mass_dic["unit"] = "g"
        self.matrix_mass_dic["thumb_mass"] = sum(sum(row) for row in dic["thumb_matrix"])
        self.matrix_mass_dic["index_mass"] = sum(sum(row) for row in dic["index_matrix"])
        self.matrix_mass_dic["middle_mass"] = sum(sum(row) for row in dic["middle_matrix"])
        self.matrix_mass_dic["ring_mass"] = sum(sum(row) for row in dic["ring_matrix"])
        self.matrix_mass_dic["little_mass"] = sum(sum(row) for row in dic["little_matrix"])
        msg.data = json.dumps(self.matrix_mass_dic)
        self.matrix_touch_pub_mass.publish(msg)

    def pub_matrix_dic(self):
        """发布矩阵数据JSON格式"""
        msg = String()
        # 尝试获取当前的 ROS 时间
        try:
            current_time = rospy.Time.now()
            # 提取 secs 和 nsecs
            t_secs = current_time.secs
            t_nsecs = current_time.nsecs
        except rospy.ROSInitException:
            # 如果 ROS 时间系统尚未启动 (例如，没有 roscore)，使用系统时间作为备用
            # 这种情况下，时间可能不够精确或与 ROS bag 时间不同步
            t_secs = int(time.time())
            t_nsecs = int((time.time() - t_secs) * 1e9)
            rospy.logwarn("ROS Time not available, using system time.")
        self.matrix_dic["stamp"]["secs"] = t_secs
        self.matrix_dic["stamp"]["nsecs"] = t_nsecs
        msg.data = json.dumps(self.matrix_dic)
        self.matrix_touch_pub.publish(msg)

    def pub_matrix_point_cloud(self):
        tmp_dic = self.matrix_dic.copy()
        del tmp_dic['stamp']               # 去掉时间戳字段
        all_matrices = list(tmp_dic.values())  # 5 帧，每帧 6×12=72 个数
        # 摊平到一维：360 个 float
        flat_list = [v for frame in all_matrices for v in frame]  # 360
        flat = np.concatenate([np.asarray(np.clip(c, 0, 255), dtype=np.uint8) for c in flat_list])
        fields = [PointField('val', 0, PointField.UINT8, 1)]
        pc = PointCloud2()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = ''   # 可改成你需要的坐标系
        pc.height = 1
        pc.width = flat.size         # 360
        pc.fields = fields
        pc.is_bigendian = False
        pc.point_step = 1            # 1 个 float32
        pc.row_step = pc.point_step * pc.width
        pc.data = flat.tobytes()     # 1440 字节
        self.matrix_touch_pub_pc.publish(pc)
        


    def signal_handler(self,sig, frame):
        #self.open_can.close_can(self.can)
        sys.exit(0)  # Exit the program normally

if __name__ == '__main__':
    '''
    本节点用于收集手指状态和压感数据。初始化速度为最大值，初始化扭矩为最大值。如需要修改可修改scripts/common/init_position.py文件，文件内有说明
    '/cb_{self.hand_type}_hand_control_cmd' 话题类型为 sensor_msgs/msg/JointState 控制话题，限制 30Hz
    /cb_{self.hand_type}_hand_state 话题类型为 sensor_msgs/msg/JointState 40Hz
    '/cb_{self.hand_type}_hand_matrix_touch' 话题类型为 std_msgs/msg/String 40Hz
    '/cb_{self.hand_type}_hand_matrix_touch_pc' 话题类型为 sensor_msgs/msg/PointCloud2 40Hz
    运行命令
    rosrun linker_hand_sdk_ros linker_hand_advanced_l6.py --hand_type right --can can0 --is_touch true
    '''
    rospy.init_node('linker_hand_advanced_l6', anonymous=True)
    parser = argparse.ArgumentParser(description='LinkerHand advanced control')
    parser.add_argument('--hand_type',   required=True, choices=['left','right'])
    parser.add_argument('--can',       required=True, help='CAN interface, e.g. can0')
    parser.add_argument('--is_touch',  type=lambda x: x.lower()=='true', required=True, help='bool: true/false')
    args=parser.parse_args()
    linker_hand = LinkerHandAdvancedL6(hand_type=args.hand_type, is_touch=args.is_touch, can=args.can)
    signal.signal(signal.SIGINT, linker_hand.signal_handler)  # Ctrl+C
    signal.signal(signal.SIGTERM, linker_hand.signal_handler)  # kill command
    embedded_version = linker_hand.embedded_version
    if embedded_version[2] < 8:
        ColorMsg(msg=f"固件版本过低，请升级固件到V{embedded_version[0]}.{embedded_version[1]}.8及以上版本", color="red")
        sys.exit(0)
    rospy.spin()
        
