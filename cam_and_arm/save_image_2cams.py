## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

save_dir = os.path.expanduser('/home/ur5/rdtkinova/cam_and_arm/save_image')
os.makedirs(save_dir, exist_ok=True) 

# 获取所有连接的设备
context = rs.context()
connected_devices = [d.get_info(rs.camera_info.serial_number) for d in context.devices]

# 为每个设备创建管道和配置
pipelines = []
for device_id in connected_devices:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(device_id)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    pipelines.append((pipeline, device_id))

print(pipelines)

# 保存图像间隔1秒
save_interval = 1.0

last_save_times = [time.time() for _ in pipelines]

try:
    while True:
        color_images = []
        depth_images = []
        
        for i, (pipeline, device_id) in enumerate(pipelines):
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())  
            color_image = np.asanyarray(color_frame.get_data())  

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 将RGB和深度图像添加到列表中
            color_images.append(color_image)
            depth_images.append(depth_colormap)

            current_time = time.time()
            if current_time - last_save_times[i] >= save_interval:
                timestamp = int(current_time*100)
                color_path = os.path.join(save_dir, f"color_{device_id}_{timestamp}.png")
                depth_path = os.path.join(save_dir, f"depth_{device_id}_{timestamp}.npy")

                cv2.imwrite(color_path, color_image)  
                np.save(depth_path, depth_image)     
                last_save_times[i] = current_time
                
                print(f"Saved color image to {color_path}")
                print(f"Saved depth image array to {depth_path}")
            
        # 将所有图像水平拼接
        if color_images and depth_images:
            combined_color_image = np.hstack(color_images)
            combined_depth_image = np.hstack(depth_images)

            combined_image = np.vstack((combined_color_image, combined_depth_image))

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', combined_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

finally:
    for pipeline, _ in pipelines:
        pipeline.stop()
    cv2.destroyAllWindows()