import pyrealsense2 as rs # to control the camera

import os

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

        # 将serial number映射到index
        self.serial_number2index = {serial_number: index for index, serial_number in enumerate(connected_devices)}
        # 将view映射到serial number
        self.view2serial_number = {
            'global': '244222076240',
            'wrist': '233522079334'
        }
        # 将serial number映射到view
        self.serial_number2view = {
            '244222076240': 'global',
            '233522079334': 'wrist'
        }
        
    
        self.global_camera = Camera(self.view2serial_number['global'])
        self.wrist_camera = Camera(self.view2serial_number['wrist'])

# camera class
class Camera:
    def __init__(self, serial_number):
        self.serial_number = serial_number


class RobotEnv:
    def __init__(self, env_id, obs_mode, control_mode, render_mode, reward_mode, 
                 sensor_configs, human_render_camera_configs, viewer_camera_configs, sim_backend):
        # 初始化真实机械臂环境
        self.env_id = env_id
        self.obs_mode = obs_mode
        self.control_mode = control_mode
        self.render_mode = render_mode
        self.reward_mode = reward_mode
        self.sensor_configs = sensor_configs
        self.human_render_camera_configs = human_render_camera_configs
        self.viewer_camera_configs = viewer_camera_configs
        self.sim_backend = sim_backend
        # 其他初始化代码
        
        # initialize camera
        # self.camera = Camera()
        # TODO: 初始化相机

        pass

    def reset(self, seed=None):

        # 重置环境到初始状态

        # 使用提供的种子来初始化随机数生成器
        if seed is not None:

            # 设置随机种子

            pass  # 具体实现取决于环境的随机性控制

        # 返回初始观测值和其他可能的初始化信息
        initial_observation = None  # 需要根据具体环境实现
        additional_info = None  # 可能的其他信息 (不需要)
        
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
        pass

    def shootImage(self):
        """
        # 拍摄图像

        # 返回值: 一张图像 np格式 来自global相机
        """
        # TODO: 拍摄图像


if __name__ == "__main__":
    camera = Camera()
