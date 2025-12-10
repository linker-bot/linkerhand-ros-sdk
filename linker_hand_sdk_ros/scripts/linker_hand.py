#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rospy, signal, rospkg, sys, os, math, time, threading, json,itertools
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

class LinkerHand:
    def __init__(self):
        self.hand_type = rospy.get_param('~hand_type', "right")
        self.hand_joint = rospy.get_param('~hand_joint', "L10")
        self.is_touch = rospy.get_param('~touch', "true")
        self.can = rospy.get_param('~can', "can0")
        self.modbus = rospy.get_param('~modbus', "None")
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
        self.hz = 1.0/60.0
        self._init_hand()
        self.thread_get_state = threading.Thread(target=self.get_state)
        self.thread_get_state.daemon = True
        self.thread_get_state.start()
        self.thread_pub_state = threading.Thread(target=self.pub_state)
        self.thread_pub_state.daemon = True
        self.thread_pub_state.start()

    def _init_hand(self):
        self.hand_setting_sub = rospy.Subscriber("/cb_hand_setting_cmd", String, self._hand_setting_cb)
        self.api = LinkerHandApi(hand_type=self.hand_type, hand_joint=self.hand_joint,can=self.can,modbus=self.modbus)
        # 获取Linker Hand版本号
        self.embedded_version = self.api.get_embedded_version()
        # 获取指尖压感类型
        self.touch_type = self.api.get_touch_type()
        self.hand_cmd_sub = rospy.Subscriber(f"/cb_{self.hand_type}_hand_control_cmd", JointState, self.hand_cmd_cb, queue_size=10)
        self.hand_state_pub = rospy.Publisher(f'/cb_{self.hand_type}_hand_state', JointState, queue_size=10)
        self.hand_state_arc_pub = rospy.Publisher(f'/cb_{self.hand_type}_hand_state_arc', JointState, queue_size=10)
        self.hand_info_pub = rospy.Publisher(f"/cb_{self.hand_type}_hand_info", String, queue_size=10)
        if self.is_touch == True:
            if self.touch_type == 2:
                self.matrix_touch_pub = rospy.Publisher(f"/cb_{self.hand_type}_hand_matrix_touch" ,String, queue_size=10)
                self.matrix_touch_pub_pc = rospy.Publisher(f"/cb_{self.hand_type}_hand_matrix_touch_pc2" ,PointCloud2, queue_size=10)
                ColorMsg(msg=f"Linker Hand {self.hand_type} {self.hand_joint} It features a matrix pressure sensor and has been enabled in the configuration",color="green")
            elif self.touch_type != -1:
                self.touch_pub = rospy.Publisher(f"/cb_{self.hand_type}_hand_touch", Float32MultiArray, queue_size=10)
                ColorMsg(msg=f"Linker Hand {self.hand_type} {self.hand_joint} It features a matrix pressure sensor and has been enabled in the configuration",color="green")
        else:
            ColorMsg(msg=f"Linker Hand {self.hand_type} {self.hand_joint} Not opened or equipped with a pressure sensor",color="yellow")
        # 1. 初始化默认值
        pose = None
        torque = [200, 200, 200, 200, 200]
        speed = [80, 200, 200, 200, 200]
        # 2. 查找配置（将手关节名称转换为大写，以匹配字典键）
        joint_key = self.hand_joint.upper()
        # 3. 处理 O6/L6/L6P 的情况 (它们共享相同的配置)
        if joint_key in ("O6", "L6", "L6P"):
            joint_key = "O6" # 将它们都映射到 "O6" 的配置上
        # 4. 获取配置并更新参数
        config_data = CONFIG.get(joint_key)
        if config_data:
            # 仅当配置中提供了 pose/torque/speed 时才更新它们
            pose = config_data.get("pose", pose)
            torque = config_data.get("torque", torque)
            speed = config_data.get("speed", speed)
        # 5. 执行操作 (保持不变)
        if pose is not None:
            # L20, G20, L21, L25 的 torque 和 speed 默认为初始的 [200...] 和 [80...]
            # 如果它们需要自定义的 torque 和 speed，您需要在 CONFIG 字典中为它们添加这些键值对。
            self.api.set_speed(speed=speed)
            self.api.set_torque(torque=torque)
            self.api.finger_move(pose=pose)

    def _hand_setting_cb(self, msg):
        data = json.loads(msg.data)
        print(f"Received setting command: {data['setting_cmd']}")
        if data["params"]["hand_type"] == "left":
            hand = self.api
            hand_left = True
        elif data["params"]["hand_type"] == "right":
            hand = self.api
            hand_right = True
        else:
            print("Please specify the hand part to be set")
            return
        self.is_lock = True
        # Set maximum torque
        if data["setting_cmd"] == "set_max_torque_limits": # Set maximum torque
            torque = list(data["params"]["torque"])
            hand.set_torque(torque=torque)
            
        if data["setting_cmd"] == "set_speed": # Set speed
            if isinstance(data["params"]["speed"], list) == True:
                speed = data["params"]["speed"]
                hand.set_speed(speed=speed)
            else:
                ColorMsg(msg=f"Speed parameter error, speed must be a list", color="red")
        if data["setting_cmd"] == "clear_faults": # Clear faults
            if hand_left == True and self.hand_joint == "L10" :
                ColorMsg(msg=f"L10 left hand cannot clear faults")
            elif hand_right == True and self.hand_joint == "L10" :
                ColorMsg(msg=f"L10 right hand cannot clear faults")
            else:
                hand.clear_faults()
        if data["setting_cmd"] == "get_faults": # Get faults
            f = hand.get_fault()
            ColorMsg(msg=f"Get faults: {f}")
        if data["setting_cmd"] == "electric_current": # Get current
            ColorMsg(msg=f"Get current: {hand.get_current()}")
        if data["setting_cmd"] == "set_electric_current": # Set current
            if isinstance(data["params"]["current"], list) == True:
                hand.set_current(data["params"]["current"])
        self.is_lock = False

    def hand_cmd_cb(self, msg):
        pose = list(msg.position)
        vel = list(msg.velocity)
        if len(pose) == 0:
            return
        else:
            self.last_position_cmd = pose
            self.last_velocity_cmd = vel

    def joint_state_msg(self, pose,vel=[]):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = []
        joint_state.position = pose
        if len(vel) > 1:
            joint_state.velocity = vel
        return joint_state
    
    def get_state(self):
        count = 0
        while True:
            if self.sdk_v == 2: # 新固件Linker Hand使用默认等待阈值
                sleep_time = 0
            else:
                sleep_time = 0.009
            if self.is_lock == False:
                """如果有命令，则执行命令动作"""
                if self.last_position_cmd != None:
                    self.api.finger_move(pose=self.last_position_cmd)
                    time.sleep(0.005)
                    if len(self.last_velocity_cmd) > 0:
                        vel = self.last_velocity_cmd
                    else:
                        vel = [0] * 6
                    if all(x == 0 for x in vel):
                        pass
                    else:
                        """如果支持实时速度控制，则改变速度"""
                        if (self.hand_joint == "O6" or self.hand_joint == "L6") and len(vel) == 6:
                            speed = vel
                            self.api.set_joint_speed(speed=speed)
                        elif self.hand_joint == "L7" and len(vel) == 7:
                            speed = vel
                            self.api.set_joint_speed(speed=speed)
                        elif self.hand_joint == "L10" and len(vel) == 10:
                            speed = [vel[0],vel[2],vel[3],vel[4],vel[5]]
                            self.api.set_joint_speed(speed=speed)
                        elif self.hand_joint == "L20" and len(vel) == 20:
                            speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                            self.api.set_joint_speed(speed=speed)
                        elif self.hand_joint == "G20" and len(vel) == 20:
                            speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                            self.api.set_joint_speed(speed=speed)
                        elif self.hand_joint == "L21" and len(vel) == 25:
                            speed = vel
                            self.api.set_joint_speed(speed=speed)
                        elif self.hand_joint == "L25" and len(vel) == 25:
                            speed = vel
                            self.api.set_joint_speed(speed=speed)
                        time.sleep(0.005)
                    # 重置命令
                    self.last_position_cmd = None
                    self.last_velocity_cmd = None
                if count % 2 == 0:
                    """获取手当前状态"""
                    if self.hand_state_pub.get_num_connections() > 0:
                        state = self.api.get_state()
                        vel = self.api.get_joint_speed()
                        self.last_hand_state['state'] = state
                        self.last_hand_state['vel'] = vel
                if count == 3 and self.is_touch == True and self.touch_type == 1 and self.touch_pub.get_num_connections() > 0:
                    """单点式压力传感器"""
                    force = self.api.get_force()
                    # 扁平化和浮点转换合并到一行
                    self.last_touch_force = [float(val) for sublist in force for val in sublist]
                if self.is_touch == True and self.touch_type == 2 and (self.matrix_touch_pub.get_num_connections() > 0 or self.matrix_touch_pub_pc.get_num_connections() > 0):
                    """矩阵式压力传感器"""
                    if count == 3:
                        self.matrix_dic["thumb_matrix"] = self.api.get_thumb_matrix_touch(sleep_time=sleep_time).tolist()
                    if count == 4:
                        self.matrix_dic["index_matrix"] = self.api.get_index_matrix_touch(sleep_time=sleep_time).tolist()
                    if count == 5:
                        self.matrix_dic["middle_matrix"] = self.api.get_middle_matrix_touch(sleep_time=sleep_time).tolist()
                    if count == 6:
                        self.matrix_dic["ring_matrix"] = self.api.get_ring_matrix_touch(sleep_time=sleep_time).tolist()
                    if count == 7:
                        self.matrix_dic["little_matrix"] = self.api.get_little_matrix_touch(sleep_time=sleep_time).tolist()
                    time.sleep(0.005)
                if count == 9 and self.hand_info_pub.get_num_connections() > 0:
                    """手部信息"""
                    self.last_hand_info = {
                        "version": self.embedded_version, # Dexterous hand version number
                        "hand_joint": self.hand_joint, # Dexterous hand joint type
                        "hand_type": self.hand_type,
                        "speed": self.api.get_speed(), # Current speed threshold of the dexterous hand
                        "current": self.api.get_current(), # Current of the dexterous hand
                        "fault": self.api.get_fault(), # Current fault of the dexterous hand
                        "motor_temperature": self.api.get_temperature(), # Current motor temperature of the dexterous hand
                        "torque": self.api.get_torque(), # Current torque of the dexterous hand
                        "is_touch":self.is_touch,
                        "touch_type": self.touch_type,
                        "finger_order": self.api.get_finger_order() # Finger motor order
                    }
                    time.sleep(0.005)
            if count == 9:
                count = 0
            count += 1
            time.sleep(0.005)
            

    def pub_state(self):
        while True:
            """发布状态"""
            if self.hand_state_pub.get_num_connections() > 0:
                state_msg = self.joint_state_msg(pose=self.last_hand_state['state'], vel=self.last_hand_state['vel'])
                self.hand_state_pub.publish(state_msg)
            """发布压感"""
            if self.is_touch == True and self.touch_type == 1 and self.touch_pub.get_num_connections() > 0:
                msg = Float32MultiArray()
                msg.data = self.last_touch_force
                self.touch_pub.publish(msg)
            if self.is_touch == True and self.touch_type == 2 and (self.matrix_touch_pub.get_num_connections() > 0 or self.matrix_touch_pub_pc.get_num_connections() > 0):
                self.pub_matrix_dic()
                self.pub_matrix_point_cloud()
            """发布配置信息"""
            if self.hand_info_pub.get_num_connections() > 0:
                msg = String()
                msg.data = json.dumps(self.last_hand_info)
                self.hand_info_pub.publish(msg)
            time.sleep(self.hz)
            
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
        flat = np.array(flat_list, dtype=np.uint8)              # (360,)

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
    rospy.init_node('linker_hand_sdk', anonymous=True)
    linker_hand = LinkerHand()
    signal.signal(signal.SIGINT, linker_hand.signal_handler)  # Ctrl+C
    signal.signal(signal.SIGTERM, linker_hand.signal_handler)  # kill command
    embedded_version = linker_hand.embedded_version
    if embedded_version == None:
        ColorMsg(msg=f"No Hand Connected", color="red")
    if linker_hand.hand_joint.upper() == "O6" or linker_hand.hand_joint.upper() == "L6" or linker_hand.hand_joint.upper() == "G20":
        ColorMsg(msg=f"New Matrix Touch For SDK V2", color="green")
        linker_hand.sdk_v=2
    elif len(embedded_version) == 3 or len(embedded_version) == 6:
        ColorMsg(msg=f"New Matrix Touch For SDK V2", color="green")
        linker_hand.sdk_v=2
    elif len(embedded_version) > 4 and ((embedded_version[0]==10 and embedded_version[4]>35) or (embedded_version[0]==7 and embedded_version[4]>50) or (embedded_version[0] == 6)):
        ColorMsg(msg=f"New Matrix Touch For SDK V2", color="green")
        linker_hand.sdk_v=2
    else:
        ColorMsg(msg=f"SDK V1", color="green")
        linker_hand.sdk_v=1
    rospy.spin()
        
