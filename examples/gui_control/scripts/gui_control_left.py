#!/usr/bin/env python3
from PyQt5.QtWidgets import QMainWindow, QSplitter, QApplication,QMessageBox,QPushButton
from PyQt5.QtCore import Qt, QTimer
import yaml, os, sys,time,json,rospkg,rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32MultiArray, String
import threading
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from views.left_view import LeftView
from views.right_view import RightView
from views.wave_form_plot import WaveformPlot
from views.temperature_plot import TemperaturePlot
from utils.ros_handler import RosHandler
from utils.load_write_yaml import LoadWriteYaml
from utils.color_msg import ColorMsg
'''
LinkerHand图形控制
'''
global package_path
# 创建 rospkg.RosPack 对象
rospack = rospkg.RosPack()
# 获取指定包的路径
package_sdk = "linker_hand_sdk_ros"
package_gui = "gui_control"
sdk_path = rospack.get_path(package_sdk)
gui_path = rospack.get_path(package_gui)
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # 初始化 ROS 节点
        rospy.init_node('gui_control_left', anonymous=True)
        self.hand_type = "left"
        self.hand_joint = rospy.get_param('~hand_joint', "L10")
        self.left_hand = True
        self.right_hand = False
        self.hand_exists = True
        
        self.rate = rospy.Rate(15)
        self.yaml = LoadWriteYaml() # 初始化配置文件
        self.set_pub = rospy.Publisher('/cb_hand_setting_cmd', String, queue_size=1)
        self.colse_sdk_sub = rospy.Subscriber("/close_sdk",String,self.close_sdk,queue_size=1)
        self.last_normal_force = [0.0] * 5
        self.last_approach_inc = [0.0] * 5
        self.motor_temperature = [0] * 10 # 电机温度
        # 读取配置文件
        #self.setting = self.yaml.load_setting_yaml()
        self._init_hand_joint()
        self._init_gui_view()
        if self.hand_joint == "L25":
            self.add_button_position = [255] * 25 # 记录添加按钮的位置
            # 要发布的最后动作序列
            self.last_position = [255] * 25
            self.set_speed(speed=[255,255,255,255,255,255])
        elif self.hand_joint == "L21":
            self.add_button_position = [255] * 25 # 记录添加按钮的位置
            # 要发布的最后动作序列
            self.last_position = [255] * 25
            self.set_speed(speed=[250,250,250,250,250])
        elif self.hand_joint == "L10":
            self.add_button_position = [255] * 10 # 记录添加按钮的位置
            # 要发布的最后动作序列
            self.last_position = [255] * 10
            self.set_speed(speed=[130,160,160,160,160])
        elif self.hand_joint == "L7":
            self.add_button_position = [255] * 7 # 记录添加按钮的位置
            # 要发布的最后动作序列
            self.last_position = [255] * 7
            self.set_speed(speed=[180,250,250,250,250,250,250])
        elif self.hand_joint == "L6":
            self.add_button_position = [255] * 6 # 记录添加按钮的位置
            # 要发布的最后动作序列
            self.last_position = [255] * 6
            self.set_speed(speed=[180,250,250,250,250,250])
        elif self.hand_joint == "L20":
            # 要发布的最后动作序列
            self.last_position = [255] * 20
            self.set_speed(speed=[180,250,250,250,250])
        if self.left_hand == True:
            self.hand_pub = rospy.Publisher("/cb_left_hand_control_cmd",JointState,queue_size=10)
        elif self.right_hand == True:
            self.hand_pub = rospy.Publisher("/cb_right_hand_control_cmd",JointState,queue_size=10)
        self.running = True
        self.stop_event = threading.Event()  # 创建一个事件对象
        self.udp_thread = threading.Thread(target=self._send_position)
        self.udp_thread.daemon = True  # 设置为守护线程
        self.udp_thread.start()
    def close_sdk(self, msg):
        data = json.loads(msg.data)
        if data["close_sdk"] == True:
            self.closeEvent(event=True)
    # 获取电机温度数据
    def get_info_data(self, info_data):
        data = json.loads(info_data.data)
        if self.left_hand == True:
            self.motor_temperature = list(data["left_hand"]["motor_temperature"])
        elif self.right_hand == True:
            self.motor_temperature = list(data["right_hand"]["motor_temperature"])
    # 获取压感数据
    def get_force_data(self,force_data):
        data = force_data.data
        self.last_normal_force = data[:5]
        self.last_approach_inc = data[10:15]
    
    def _send_position(self):    
        while not rospy.is_shutdown():
            positons = self.create_joint_state_msg(position=self.last_position,names=[])
            self.hand_pub.publish(positons)
            self.rate.sleep()
            #rospy.loginfo("写出的数据:%s",positons)


    def _init_hand_joint(self):
        self.yaml = LoadWriteYaml() # 初始化配置文件
        # 读取配置文件
        self.setting = self.yaml.load_setting_yaml()
        # # 判断左手是否配置
        # self.left_hand = False
        # self.right_hand = False
        # if self.setting['LINKER_HAND']['LEFT_HAND']['EXISTS'] == True:
        #     self.left_hand = True
        # elif self.setting['LINKER_HAND']['RIGHT_HAND']['EXISTS'] == True:
        #     self.right_hand = True
        # # gui控制只支持单手，这里进行左右手互斥
        # if self.left_hand == True and self.right_hand == True:
        #     self.left_hand = True
        #     self.right_hand = False
        # if self.left_hand == True:
        #     print("左手")
        #     self.hand_exists = True
        #     self.hand_joint = self.setting['LINKER_HAND']['LEFT_HAND']['JOINT']
        #     self.hand_type = "left"
        # if self.right_hand == True:
        #     print("右手")
        #     self.hand_exists = True
        #     self.hand_joint = self.setting['LINKER_HAND']['RIGHT_HAND']['JOINT']
        #     self.hand_type = "right"

        
        self.init_pos = [255] * 10
        # if self.hand_joint == "L25":
        #     # L25
        #     self.init_pos = [255] * 24
        #     self.joint_name = ["拇指根部", "食指根部", "中指根部", "无名指根部","预留","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","预留","拇指横摆","预留","预留","预留","预留","拇指尖部","食指尖部","中指尖部","无名指部","预留","预留","预留","预留","预留"]
        if self.hand_joint == "L25":
            self.set_enable()
            # L25
            self.init_pos = [255] * 25
            # topic
            self.joint_name = ["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部","中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]
            #self.joint_name = ["拇指根部0", "食指根部1", "中指根部2", "无名指根部3","小指根部4","拇指侧摆5","食指侧摆6","中指侧摆","无名指侧摆8","小指侧摆9","拇指横摆10","预留","预留","预留","预留","拇指中部15","食指中部16","中指中部17","无名指中部18","小指中部19","拇指指尖20","食指指尖21","中指指尖22","无名指指尖23","小指指尖24"]
        elif self.hand_joint == "L21":
            self.set_enable()
            # L25
            self.init_pos = [255] * 25
            # topic
            self.joint_name = ["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部","大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆","大拇指横滚","预留","预留","预留","预留","大拇指中部","预留","预留","预留","预留","大拇指指尖","食指指尖","中指指尖","无名指指尖","小拇指指尖"]
            #self.joint_name = ["拇指根部0", "食指根部1", "中指根部2", "无名指根部3","小指根部4","拇指侧摆5","食指侧摆6","中指侧摆","无名指侧摆8","小指侧摆9","拇指横摆10","预留","预留","预留","预留","拇指中部15","食指中部16","中指中部17","无名指中部18","小指中部19","拇指指尖20","食指指尖21","中指指尖22","无名指指尖23","小指指尖24"]
        elif self.hand_joint == "L20":
            self.init_pos = [255,255,255,255,255,255,10,100,180,240,245,255,255,255,255,255,255,255,255,255]
            # L20
            self.joint_name = ["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部","拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆","拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端","中指末端","无名指末端","小指末端"]
        elif self.hand_joint == "L10":
            # L10
            self.init_pos = [255] * 10
            self.joint_name = ["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部","小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"]
        elif self.hand_joint == "L7":
            # L7
            self.init_pos = [250] * 7
            self.joint_name = ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲","拇指旋转"]
        elif self.hand_joint == "L6":
            # L6
            self.init_pos = [250] * 6
            self.joint_name = ["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲","小拇指弯曲"]
        
    
    # 初始化窗口界面
    def _init_gui_view(self):
        if self.hand_type == "left":
            self.setWindowTitle(f"Linker_Hand:左手- {self.hand_joint} Control - Qt5 with ROS")
        else:
            self.setWindowTitle(f"Linker_Hand:右手- {self.hand_joint} Control - Qt5 with ROS")
        self.setGeometry(100, 100, 600, 800)
        # 创建分割线
        splitter = QSplitter(Qt.Horizontal)
        splitter.setStyleSheet("""
            QSplitter::handle {
                width:1px;
                background-color: lightgray;
                margin: 15px 20px;
            }
        """)
        # 左侧滑动条界面
        self.left_view = LeftView(joint_name=self.joint_name, init_pos=self.init_pos,hand_type=self.hand_type)
        splitter.addWidget(self.left_view)
        self.left_view.slider_value_changed.connect(self.handle_slider_value_changed)
        # 右侧记录动作界面
        self.right_view = RightView(hand_joint=self.hand_joint, hand_type=self.hand_type)
        splitter.addWidget(self.right_view)
        # 接收到信号槽事件，这里用于记录动作序列更新滑动条数据
        self.right_view.handle_button_click.connect(self.handle_button_click)
        self.right_view.add_button_handle.connect(self.add_button_handle)
        splitter.setSizes([600, 450])
        self.setCentralWidget(splitter)
    # 初始化波形图
    def _init_normal_force_plot(self): 
        # 初始化波形图
        self.normal_force_plot = WaveformPlot(num_lines=5, labels=["thumb","index finger","middle finger","ring finger","little finger"],title="法向压力波形图")
        # 设置波形图位置
        self.normal_force_plot.setGeometry(700, 100, 800, 400)
        self.normal_force_plot.show()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_normal_force_plot)
        self.timer.start(50)
    def _init_approach_inc_plot(self):
        # 初始化波形图
        self.approach_inc_plot = WaveformPlot(num_lines=5, labels=["thumb","index finger","middle finger","ring finger","little finger"],title="接近感应波形图")
        # 设置波形图位置
        self.approach_inc_plot.setGeometry(700, 600, 800, 400)
        self.approach_inc_plot.show()
        self.timer2 = QTimer()
        self.timer2.timeout.connect(self.update_approach_inc_plot)
        self.timer2.start(50)
    def _init_temperature_plot(self):
        # 初始化温度波形图
        self.temperature_plot = TemperaturePlot(num_lines=10, labels=None, title="电机温度波形图")
        self.temperature_plot.setGeometry(700, 1000, 800, 400)
        self.temperature_plot.show()
        self.timer3 = QTimer()
        self.timer3.timeout.connect(self.update_temperature_plot)
        self.timer3.start(50)
    # 点击按钮后将动作数值写入yaml文件
    def handle_button_click(self,text):
        all_action = self.yaml.load_action_yaml(hand_type=self.hand_type,hand_joint=self.hand_joint)
        for index,pos in enumerate(all_action):
            if pos['ACTION_NAME'] == text:
                position = pos['ACTION_POS']
                print(type(position))
        ColorMsg(msg=f"动作名称:{text}, 动作数值:{position}", color="green")
        self.last_position = position
        self.left_view.set_slider_values(values=position)

    #点击添加按钮后将动作数值写入yaml文件
    def add_button_handle(self,text):
        self.add_button_position = self.left_view.get_slider_values()
        self.add_button_text = text
        self.yaml.write_to_yaml(action_name=text, action_pos=self.left_view.get_slider_values(),hand_joint=self.hand_joint,hand_type=self.hand_type)


    # 通过信号机制实时获取滑动条的当前值
    def handle_slider_value_changed(self, slider_values):
        #print("实时获取滑动条的当前值:", slider_values)
        slider_values_list = []
        for key in slider_values:
            slider_values_list.append(slider_values[key])
        self.last_position = slider_values_list

    # 更新滑动条状态
    def update_label(self, index, value):
        self.left_view.labels[index].setText(f"{self.joint_name[index]}: {value}")

    def create_joint_state_msg(self, position, names):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        msg.position = list(map(float, position))
        msg.velocity = [0.0] * len(position)
        msg.effort = [0.0] * len(position)
        return msg
    # 更新法向压力波形图
    def update_normal_force_plot(self): 
        import random
        #values = self.api.get_force() # 获取法相压力数据
        self.normal_force_plot.update_data(self.last_normal_force)
    # 更新接近感应波形图
    def update_approach_inc_plot(self):
        import random
        converted_list = []
        # 遍历原始列表并进行转换
        for value in self.last_approach_inc:
            # 由于值是从 255 递减到 0，所以 255 - value 就能得到 0 到 255 的范围
            new_value = 255 - value
            converted_list.append(new_value)
        self.approach_inc_plot.update_data(converted_list)
    # 更新温度波形图
    def update_temperature_plot(self):
        self.temperature_plot.update_data(self.motor_temperature)
    def set_speed(self,speed=[130,250,250,250,250]):
        msg = String()
        cmd = {
            "setting_cmd":"set_speed",
            "params":{
                "hand_type": self.hand_type,
                "speed":speed,
            }
        }
        msg.data = json.dumps(cmd)
        for i in range(3):
            self.set_pub.publish(msg)
            time.sleep(0.1)
        ColorMsg(msg=f"设置速度:{speed}", color="green")
    def set_enable(self):
        '''设置LinkerHand处于使能状态'''
        m = String()
        d = {
            "setting_cmd":"set_enable",
            "params":{
                "hand_type":self.hand_type
            }
        }
        m.data = json.dumps(d)
        for i in range(3):
            self.set_pub.publish(m)
            time.sleep(0.1)
    def set_disability(self):
        '''设置LinkerHand处于失能状态'''
        m = String()
        d = {
            "setting_cmd":"set_enable",
            "params":{
                "hand_type":self.hand_type
            }
        }
        m.data = json.dumps(d)
        self.set_pub.publish(m)
    # 关闭窗口结束程序
    def closeEvent(self, event):
        """关闭窗口时停止线程并释放资源"""
        self.stop_event.set()  # 设置事件，通知线程停止
        self.running = False
        # 关闭波形图窗口
        #self.normal_force_plot.close()
        #self.approach_inc_plot.close()
        #self.temperature_plot.close()
        self.close()
        #event.accept()

    

    

# 主程序运行
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())
