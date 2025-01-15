import pyrealsense2 as rs # to control the camera
import numpy as np
import os

import time

import logging

import sys
import logging
import Servoj_RTDE_UR5.rtde.rtde as rtde
import Servoj_RTDE_UR5.rtde.rtde_config as rtde_config
import time
from Servoj_RTDE_UR5.min_jerk_planner_translation import PathPlanTranslation

import serial # 控制夹爪使用的串口通信

# Manager to control the cameras
class Cameras:
    def __init__(self):
        # directory to save the images
        save_dir = os.path.expanduser('/home/ur5/rdtkinova/cam_and_arm/save_image')
        os.makedirs(save_dir, exist_ok=True) 

        # 获取所有连接的设备
        context = rs.context()
        # 所有可连接设备的serial number
        connected_devices = [d.get_info(rs.camera_info.serial_number) for d in context.devices]
        print('Connected devices:', connected_devices)

        # 将serial number映射到index
        self.serial_number2index = {serial_number: index for index, serial_number in enumerate(connected_devices)}
        # 将view映射到serial number
        self.view2serial_number = {
            'global': connected_devices[0],
            'wrist': connected_devices[1]
        }
        # 将serial number映射到view
        self.serial_number2view = {
            connected_devices[0]: 'global',
            connected_devices[1]: 'wrist'
        }
        
        # initialize cameras
        print('Initializing cameras...')
        self.global_camera = Camera(self.view2serial_number['global'])
        self.wrist_camera = Camera(self.view2serial_number['wrist'])
        time.sleep(2.5) # 等待相机初始化完成
        print('done')
        # 将view映射到camera对象
        self.view2camera = {
            'global': self.global_camera,
            'wrist': self.wrist_camera
        }

# camera class
class Camera:
    def __init__(self, serial_number):
        self.serial_number = serial_number

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.serial_number)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.pipeline.start(config)
        
    def get_rgb_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data()) 
        return color_image

    def get_depth_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image


class RobotEnv:
    def __init__(self, env_id, obs_mode, control_mode, render_mode, reward_mode, 
                 sensor_configs, human_render_camera_configs, viewer_camera_configs, sim_backend, ip, port):
        

        # 初始化真实机械臂环境

        # 一些是仿真环境相关，没用
        self.env_id = env_id
        self.obs_mode = obs_mode
        self.control_mode = control_mode
        self.render_mode = render_mode
        self.reward_mode = reward_mode
        self.sensor_configs = sensor_configs
        self.human_render_camera_configs = human_render_camera_configs
        self.viewer_camera_configs = viewer_camera_configs
        self.sim_backend = sim_backend


        # 连接机器人    
        self.ip = ip
        self.port = port

        # 初始化机械臂
        self.robot = ur5Robot(ip, port)
        
        # 其他初始化代码
        
        # initialize camera
        self.cameras = Cameras()
        self.view2camera = self.cameras.view2camera # 将view映射到camera对象

        self.global_camera = self.view2camera['global']
        self.wrist_camera = self.view2camera['wrist']

        #？？？pass干什么的？
        pass

    def reset(self, seed=None):

        # 重置环境到初始状态

        #？？？真实环境的reset需要随机种子？
        # 使用提供的种子来初始化随机数生成器
        if seed is not None:
            # 设置随机种子
            pass  # 具体实现取决于环境的随机性控制


        # 返回初始观测值和其他可能的初始化信息
        initial_observation = None  # 需要根据具体环境实现
        additional_info = None  # 可能的其他信息 (外面的调用不需要)
        
        return initial_observation, additional_info

    def get_state(self):
        # 获取当前状态
        # 返回值: 当前状态
        pass

    def sample_action(self):
        # 采样一个动作
        # 返回值: 动作
        pass

    def step(self, action):
        # 执行动作并返回新的观测值、奖励、是否终止、是否截断和附加信息

        # 根据action执行动作（核心）




        # 更新环境状态

        #返回观测值
        obs = {
            'agent': {
                'qpos': None  # 需要根据具体环境实现，应该是一个包含关节位置的数组
            }
        }

        #没用，不用管
        reward = 0.0  # 奖励值，需要根据具体环境实现 （没用）

        #是否在这一步终止执行
        terminated = False  
        truncated = False  

        #提示是否成功
        info = {
            'success': False  # 是否成功，需要根据具体环境实现
        }

        # 具体的动作执行和状态更新逻辑需要根据真实机械臂环境实现
        return obs, reward, terminated, truncated, info

    def render(self):
        # 渲染当前环境状态

        # 返回值: 一张图像 来自global相机
        return self.global_camera.get_rgb_frame()

    def shootImage(self):
        """
        # 拍摄图像

        # 返回值: 一张图像 np格式 来自global相机
        """
        return self.global_camera.get_rgb_frame()

class ur5Robot:
    def __init__(self, ip='192.168.0.201', port=30004, FREQUENCY=500,
                 config_filename='Servoj_RTDE_UR5/control_loop_configuration.xml'):
        # UR5机械臂的基本参数

        # 连接机器人
        self.ip = ip
        self.port = port
        
        #?
        self.joint_num = 6  # UR5有6个关节

        # 当前关节位置  
        self.current_joint_positions = None
        # 当前末端执行器位姿
        self.current_tcp_pose = None

        # 夹爪打开/关闭的命令
        self.MOTOR_OPEN_LIST = (0x02,0x00,0x20,0x49,0x20,0X00,0xC8) #机械爪松开(具体解释见机械爪用户手册)
        self.MOTOR_CLOSE_LIST = (0x02,0x01,0x20,0x49,0x20,0X00,0xC8)    #机械爪闭合，45字节是角度
        # 夹爪控制串口
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        # 夹爪状态
        self.gripper_state = 0.0 # 0.0表示夹爪打开，1.0表示夹爪关闭
        # 默认先打开夹爪
        self.ser.write(self.MOTOR_OPEN_LIST)
        # 频率
        self.FREQUENCY = FREQUENCY

        # 日志
        logging.getLogger().setLevel(logging.INFO)

        # 读取配置文件
        conf = rtde_config.ConfigFile(config_filename)
        state_names, state_types = conf.get_recipe('state')
        self.setp_names, self.setp_types = conf.get_recipe('setp')
        self.watchdog_names, self.watchdog_types = conf.get_recipe('watchdog')

        # 连接
        self.con = rtde.RTDE(self.ip, self.port)
        connection_state = self.con.connect()
        while connection_state != 0:
            time.sleep(0.5)
            connection_state = self.con.connect()
        print("---------------Successfully connected to the robot-------------\n")

        #通信
        self.con.get_controller_version()
        self.con.send_output_setup(self.setp_names, self.setp_types, self.FREQUENCY)
        self.setp = self.con.send_input_setup(self.setp_names, self.setp_types)
        self.watchdog = self.con.send_input_setup(self.watchdog_names, self.watchdog_types)

        #初始化寄存器
        self.setp.input_double_register_0 = 0
        self.setp.input_double_register_1 = 0
        self.setp.input_double_register_2 = 0
        self.setp.input_double_register_3 = 0
        self.setp.input_double_register_4 = 0
        self.setp.input_double_register_5 = 0
        self.setp.input_bit_registers0_to_31 = 0

        #连不上就退出
        if not self.con.send_start():
            sys.exit()   
         

    
    def move(self, target_pose, config_filename, trajectory_time=2, speed=0.25, acceleration=0.5):
        """
        笛卡尔空间直线运动
        Args:
            target_pose: 目标位姿 [x, y, z, rx, ry, rz]
            trajectory_time: 运动时间 (s)

            speed: 运动速度 (m/s)
            acceleration: 加速度 (m/s^2)
        """

        logging.getLogger().setLevel(logging.INFO)



        pass
    
    @staticmethod
    def list_to_setp(setp, list):
        """
        将列表转换为setp, 输入的setp是自己的setp属性
        """
        for i in range(6):
            setp.__dict__[f"input_double_register_{i}"] = list[i]
        return setp

    def stop(self):
        """
        立即停止机械臂运动
        """

        self.con.send_pause()
        self.con.disconnect()

        pass

#？？这个类下面的其他方法有没有用？

    # def move_j(self, joint_positions, speed=1.0, acceleration=1.0):
    #     """
    #     关节空间运动
    #     Args:
    #         joint_positions: 目标关节角度 [j1, j2, j3, j4, j5, j6]
    #         speed: 运动速度比例 (0.0-1.0)
    #         acceleration: 加速度比例 (0.0-1.0)
    #     """
    #     pass



    # def move_p(self, target_pose, speed=0.25, acceleration=0.5, blend_radius=0.05):
    #     """
    #     笛卡尔空间点到点运动（带圆弧过渡）
    #     Args:
    #         target_pose: 目标位姿 [x, y, z, rx, ry, rz]
    #         speed: 运动速度 (m/s)
    #         acceleration: 加速度 (m/s^2)
    #         blend_radius: 圆弧过渡半径 (m)
    #     """
    #     pass

    def get_joint_positions(self):
        """
        获取当前关节角度
        Returns:
            list: 当前关节角度 [j1, j2, j3, j4, j5, j6]
        """
        state = self.con.receive() # 获取当前状态
        current_joint_positions = state.actual_q # 获取当前关节角度
        return current_joint_positions

    def get_tcp_pose(self):
        """
        获取当前末端执行器位姿
        Returns:
            list: 当前TCP位姿 [x, y, z, rx, ry, rz]
        """
        state = self.con.receive() # 获取当前状态
        current_tcp_pose = state.actual_TCP_pose # 获取当前末端执行器位姿
        return current_tcp_pose

    def control_gripper(self, open_state, speed=1.0):
        """
        控制夹爪
        Args:
            open_state: 0.0表示打开，1.0表示关闭
            speed: 夹爪速度 (0-1)
        """
        prev_state = self.gripper_state # 获取当前夹爪状态
        self.gripper_state = open_state # 设置夹爪新状态

        # 夹爪状态发生变化时，发送命令
        if prev_state != self.gripper_state:
            if self.gripper_state == 0.0:
                self.ser.write(self.MOTOR_OPEN_LIST)
            else:
                self.ser.write(self.MOTOR_CLOSE_LIST)


    def set_tcp(self, tcp_offset):
        """
        设置工具中心点偏移
        Args:
            tcp_offset: TCP偏移量 [x, y, z, rx, ry, rz]
        """
        pass

    def set_payload(self, weight, cog=None):
        """
        设置负载
        Args:
            weight: 负载重量(kg)
            cog: 重心位置 [x, y, z]
        """
        pass

if __name__ == "__main__":
    # import cv2
    # cameras = Cameras()

    # # 测试拍摄图像
    # global_camera = cameras.view2camera['global']
    # wrist_camera = cameras.view2camera['wrist']
    # while True:
    #     global_rgb_frame = global_camera.get_rgb_frame()
    #     wrist_rgb_frame = wrist_camera.get_rgb_frame()

    #     combined_frame = np.hstack((global_rgb_frame, wrist_rgb_frame))

    #     cv2.imshow('RGB Frame', combined_frame)
    #     key = cv2.waitKey(1) & 0xFF
    #     if key == ord('q'):
    #         break
    # cv2.destroyAllWindows()

    robot = ur5Robot()
    # for i in range(10):
    #     robot.control_gripper(1.0)
    #     time.sleep(2)
        # robot.control_gripper(0.0)
        # time.sleep(2)