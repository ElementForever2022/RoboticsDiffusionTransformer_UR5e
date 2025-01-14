## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

save_dir = os.path.expanduser('/home/ur5/rdtkinova/cam_and_arm/save_image')
os.makedirs(save_dir, exist_ok=True) 

pipeline = rs.pipeline()
config = rs.config()

# select specific camera
config.enable_device('244222076140') # device 0
# config.enable_device('233522079334') # device 1

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

#save image interval 1 second 
save_interval = 1.0 
last_save_time = time.time()

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())  
        color_image = np.asanyarray(color_frame.get_data())  

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image,
                                             dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                             interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        current_time = time.time()
        if current_time - last_save_time >= save_interval:
            timestamp = int(current_time)
            color_path = os.path.join(save_dir, f"color_{timestamp}.png")
            depth_path = os.path.join(save_dir, f"depth_{timestamp}.npy")

            cv2.imwrite(color_path, color_image)  
            np.save(depth_path, depth_image)      
            
            print(f"Saved color image to {color_path}")
            print(f"Saved depth image array to {depth_path}")

            last_save_time = current_time 

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
