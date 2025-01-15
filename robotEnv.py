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

# from docs.test_6drot import compute_ortho6d_from_rotation_matrix
# from docs.test_6drot import convert_euler_to_rotation_matrix
from docs.test_6drot import compute_rotation_matrix_from_ortho6d
from docs.test_6drot import convert_rotation_matrix_to_euler


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
        #输入：action，模型生成的。3位末端位置，6位末端旋转，1位夹爪状态
    

        #TODO：将action拆解为3部分，末端位置，末端旋转（ortho6d），夹爪状态


        #6位末端旋转-》3位末端旋转（UR5要使用）
        ortho6d = [[-0.14754878, -0.98858915,  0.0303456,  -0.46593792,  0.09654019,  0.87953501]]#仅做测试，代替从模型的输入
        print(f"6D Rotation: {ortho6d}")
        rotmat_recovered = compute_rotation_matrix_from_ortho6d(ortho6d)
        euler_recovered = convert_rotation_matrix_to_euler(rotmat_recovered)
        print(f"Recovered Euler angles: {euler_recovered}")


        #TODO:合并3位末端位置和3位末端旋转为target_pose


        # 根据末端执行器位姿（3位末端位置，3位末端旋转），移动机械臂
        self.robot.move(target_pose)

        # TODO:根据夹爪状态，控制夹爪


        #TODO:获取并返回观测值
        
        #返回观测值
        obs = {
            'agent': {
                'qpos': None  # 需要根据具体环境实现，应该是一个包含关节位置的数组
            }
        }


        #没用，不用管
        reward = 0.0  # 奖励值，需要根据具体环境实现 （没用）
        #没用
        #是否在这一步终止执行
        terminated = False  
        truncated = False  
        # 没用
        #提示是否成功
        #'success'的初始值是False，判断成功后设为True
        info = {
            'success': False  
        }


        # 目前只有obs有用
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
    def __init__(self, ip='192.168.1.201', port=30004, FREQUENCY=500,
                 config_filename='Servoj_RTDE_UR5/control_loop_configuration.xml'):
        # UR5机械臂的基本参数

        # 连接机器人
        self.ip = ip
        self.port = port
        
        #?没用
        self.joint_num = 6  # UR5有6个关节

        # 当前关节位置  
        self.current_joint_positions = None
        # 当前末端执行器位姿
        self.current_tcp_pose = None

        
        # 频率
        self.FREQUENCY = FREQUENCY

        # 日志
        logging.getLogger().setLevel(logging.INFO)

        # 读取配置文件
        conf = rtde_config.ConfigFile(config_filename)
        self.state_names, self.state_types = conf.get_recipe('state')
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
        self.con.send_output_setup(self.state_names, self.state_types, self.FREQUENCY)  
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
         

    
    def move(self, target_pose, trajectory_time=2, speed=0.25, acceleration=0.5):
        """
        笛卡尔空间直线运动
        Args:
            target_pose: 目标位姿 [x, y, z, rx, ry, rz]
            trajectory_time: 运动时间 (s)

            speed: 运动速度 (m/s)
            acceleration: 加速度 (m/s^2)
        """

        #获取当前末端执行器位姿
        state = self.con.receive()
        tcp = state.actual_TCP_pose
        orientation_const = tcp[3:]

        #打印
        print("Current TCP pose:", tcp)
        print("-------Executing servoJ to point 1 -----------\n")
                
        #指定伺服模式
        self.watchdog.input_int_register_0 = 2
        self.con.send(self.watchdog)

        # 路径规划器
        planner = PathPlanTranslation(tcp, target_pose, trajectory_time)

        # 动
        t_start = time.time()
        while time.time() - t_start < trajectory_time:
            state = self.con.receive()
            t_current = time.time() - t_start

            if state.runtime_state > 1 and t_current <= trajectory_time:
                position_ref, lin_vel_ref, acceleration_ref = planner.trajectory_planning(t_current)
                pose = position_ref.tolist() + orientation_const
                self.list_to_setp(self.setp, pose)
                self.con.send(self.setp)

        # 打印
        print(f"It took {time.time()-t_start}s to execute the servoJ to point 1")
        print('Final TCP pose:', self.con.receive().actual_TCP_pose)

        self.stop()

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

class Gripper:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout) # control the gripper

        # 夹爪速度
        self.MAX_SPEED_42 = (0X01,0X40) # 最快速度为42rad/s， 也就是01A0
        self.AVER_SPEED_10 = (0X00,0XC8) # 平均速度为10rad/s， 也就是00C8

        # 夹爪打开/关闭的命令
        self.BYTE_OPEN = 0x00
        self.BYTE_CLOSE = 0x01
        # self.MOTOR_OPEN_LIST = (0x02,self.BYTE_OPEN,0x20,0x49,0x20,*self.AVER_SPEED_10) #机械爪松开(最快速度为42rad/s， 也就是01A0)
        # self.MOTOR_CLOSE_LIST = (0x02,self.BYTE_CLOSE,0x20,0x49,0x20,*self.AVER_SPEED_10) #机械爪闭合

        # 机械爪90%张开闭合
        self.MOTOR_OPEN_LIST = (0x02,self.BYTE_OPEN,0x20,0x43,0x14,*self.AVER_SPEED_10) #机械爪松开(最快速度为42rad/s， 也就是01A0)
        self.MOTOR_CLOSE_LIST = (0x02,self.BYTE_CLOSE,0x20,0x43,0x14,*self.AVER_SPEED_10) #机械爪闭合
        
        # 夹爪状态
        self.gripper_state = 0.0 # 0.0表示夹爪打开，1.0表示夹爪关闭
        # 工作状态
        self.working_state = 'ready' # 工作状态：ready表示待机，working表示工作
        # 任务状态
        self.task_start = None # 任务开始的闭合程度
        self.task_end = None # 任务计划结束的闭合程度

        # 任务时间管理
        self.task_start_time = None # 任务开始时间

        # 默认先打开夹爪
        self.ser.write(self.MOTOR_OPEN_LIST)
        # 等待夹爪打开
        time.sleep(2)
        print("Gripper initialized")
    
    def __del__(self):
        self.ser.write(self.MOTOR_CLOSE_LIST)
        self.ser.close()

    @staticmethod
    def rad2byte(rad):
        """
        将角度转换为字节
        输出为高低位的10进制
        """
        output_dec = round(rad/3.14*180*10)
        output_hex = format(output_dec, '04X') # 长度为4字符，使用0填充，大写16进制
        high_byte = output_hex[:2]
        low_byte = output_hex[2:]
        return int(high_byte, 16), int(low_byte, 16)
    @staticmethod
    def gripper_state2rad(state):
        """
        将夹爪状态转换为弧度
        """
        return state * 3.14*2*5.2

    def open(self):
        self.__update_gripper_state()
        if self.gripper_state == 1.0: # 如果夹爪状态为1.0，则夹爪已经闭合

            if self.working_state == 'ready': # 如果工作状态为ready，则夹爪可以工作
                self.working_state = 'working'
                self.task_start = self.gripper_state
                self.task_end = 0.0
                self.task_start_time = time.time() # 任务开始时间
                print('opening')
                # self.ser.write(self.MOTOR_OPEN_LIST)
                self.__open_byte(self.rad2byte(self.gripper_state2rad(0.8)))
                # time.sleep(2)
            else:
                return
        elif self.gripper_state == 0.5:
            if self.working_state == 'ready': # 如果工作状态为ready，则夹爪可以工作
                self.working_state = 'working'
                self.task_start = self.gripper_state
                self.task_end = 0.0
                self.task_start_time = time.time() # 任务开始时间
                print('opening')
                # self.ser.write(self.MOTOR_OPEN_LIST)
                self.__open_byte(self.rad2byte(self.gripper_state2rad(0.4)),speedType='AVER')
                # time.sleep(2)
            else:
                return
        else:
            # 如果夹爪状态为0.0，则夹爪已经打开
            print('gripper is already opened')
            return
        
    def grasp(self):
        self.__update_gripper_state()
        if self.gripper_state == 0.0: # 如果夹爪状态为0.0，则夹爪已经打开
            if self.working_state == 'ready': # 如果工作状态为ready，则夹爪可以工作
                self.working_state = 'working'
                self.task_start = self.gripper_state
                self.task_end = 0.5
                self.task_start_time = time.time() # 任务开始时间
                print('grasping from open')
                # self.ser.write(self.MOTOR_OPEN_LIST)
                self.__close_rad(self.gripper_state2rad(0.4),speedType='AVER')
                # time.sleep(2)
            else:
                return
        elif self.gripper_state == 1.0:
            if self.working_state == 'ready':
                self.working_state = 'working'
                self.task_start = self.gripper_state
                self.task_end = 0.5
                self.task_start_time = time.time() # 任务开始时间
                print('grasping from close')
                # self.ser.write(self.MOTOR_OPEN_LIST)
                self.__open_rad(self.gripper_state2rad(0.4),speedType='AVER')
                # time.sleep(2)
            else:
                return
        else:
            # 如果夹爪状态为0.0，则夹爪已经打开
            return


    def close(self):
        self.__update_gripper_state()
        if self.gripper_state == 0.0: # 如果夹爪状态为1.0，则夹爪已经闭合
            if self.working_state == 'ready': # 如果工作状态为ready，则夹爪可以工作
                self.working_state = 'working'
                self.task_start = self.gripper_state
                self.task_end = 1.0
                self.task_start_time = time.time() # 任务开始时间
                print('closing')
                # self.ser.write(self.MOTOR_CLOSE_LIST)
                self.__close_byte(self.rad2byte(self.gripper_state2rad(0.8)))
                # time.sleep(2)
            else:
                return
        elif self.gripper_state == 0.5:
            if self.working_state == 'ready': # 如果工作状态为ready，则夹爪可以工作
                self.working_state = 'working'
                self.task_start = self.gripper_state
                self.task_end = 1.0
                self.task_start_time = time.time() # 任务开始时间
                print('closing')
                # self.ser.write(self.MOTOR_CLOSE_LIST)
                self.__close_byte(self.rad2byte(self.gripper_state2rad(0.4)),speedType='AVER')
                # time.sleep(2)
            else:
                return
        else:
            print('gripper is already closed')
            return
        
    def __update_gripper_state(self):
        """
        更新夹爪状态
        """
        current_time = time.time()
        if self.task_start_time is not None:
            if current_time - self.task_start_time >= 3:
                self.working_state = 'ready'
                if self.task_end == 0.0:
                    self.gripper_state = 0.0
                elif self.task_end == 0.5:
                    self.gripper_state = 0.5
                else:
                    self.gripper_state = 1.0
                self.task_start = None
                self.task_end = None
                self.task_start_time = None
        
    
    def __close_rad(self, rad,speedType='MAX'):
        """
        闭合指定角度
        Args:
            rad: 角度
            speedType: 速度类型，'MAX'表示最大速度，'AVER'表示平均速度
        """
        high_byte, low_byte = self.rad2byte(rad)    
        if speedType == 'MAX':
            self.ser.write((0x02,self.BYTE_CLOSE,0x20,high_byte,low_byte,*self.MAX_SPEED_42))
        elif speedType == 'AVER':
            self.ser.write((0x02,self.BYTE_CLOSE,0x20,high_byte,low_byte,*self.AVER_SPEED_10))

    def __open_rad(self, rad,speedType='MAX'):
        """
        打开指定角度
        Args:
            rad: 角度
            speedType: 速度类型，'MAX'表示最大速度，'AVER'表示平均速度
        """
        high_byte, low_byte = self.rad2byte(rad)
        if speedType == 'MAX':
            self.ser.write((0x02,self.BYTE_OPEN,0x20,high_byte,low_byte,*self.MAX_SPEED_42))
        elif speedType == 'AVER':
            self.ser.write((0x02,self.BYTE_OPEN,0x20,high_byte,low_byte,*self.AVER_SPEED_10))

    def __close_byte(self, bytes,speedType='MAX'):
        """
        闭合指定字节
        Args:
            bytes: 字节
            speedType: 速度类型，'MAX'表示最大速度，'AVER'表示平均速度
        """
        if speedType == 'MAX':
            self.ser.write((0x02,self.BYTE_CLOSE,0x20,*bytes,*self.MAX_SPEED_42))
        elif speedType == 'AVER':
            self.ser.write((0x02,self.BYTE_CLOSE,0x20,*bytes,*self.AVER_SPEED_10))

    def __open_byte(self, bytes,speedType='MAX'):
        """
        打开指定字节
        Args:
            bytes: 字节
            speedType: 速度类型，'MAX'表示最大速度，'AVER'表示平均速度
        """
        if speedType == 'MAX':
            self.ser.write((0x02,self.BYTE_OPEN,0x20,*bytes,*self.MAX_SPEED_42))
        elif speedType == 'AVER':
            self.ser.write((0x02,self.BYTE_OPEN,0x20,*bytes,*self.AVER_SPEED_10))


if __name__ == "__main__":


    # robot = ur5Robot('192.168.0.201')
    # target_pose = [-0.503, -0.0088, 0.31397, 1.266, -2.572, -0.049]
    # robot.move(target_pose,trajectory_time=8)

    gripper = Gripper()
    print(gripper.rad2byte(5.3*3.14*2*0.9))
    time.sleep(5)
    gripper.grasp()
    print(0)


    # high_byte, low_byte = gripper.rad2byte(1800)
    time.sleep(5)
    gripper.close()
    print(1)
    time.sleep(5)
    gripper.open()
    print(2)
    time.sleep(5)
    gripper.close()
    print(3)
    time.sleep(5)
    gripper.open()
    print(4)
    time.sleep(5)

    for i in range(10):
        gripper.close()
        time.sleep(5)
        gripper.grasp()
        time.sleep(5)
        gripper.open()
        time.sleep(5)
        gripper.grasp()
        time.sleep(5)
        gripper.close()
        time.sleep(5)

    # target_pose = [-0.503, -0.0088, 0.31397, 1.266, -2.572, -0.049]
    # robot.move(target_pose,trajectory_time=8)
