#!/usr/bin/env python
# Copyright (c) 2016-2022, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys

sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
# ---------------------------------
import rtde.csv_reader as csv_reader

with open("test.csv") as csvfile:
    r = csv_reader.CSVReader(csvfile)

# 检查是否存在 actual_TCP_pose 列
if hasattr(r, "actual_TCP_pose_0"):
    # 读取 actual_TCP_pose 列的数据
    actual_TCP_pose_0_data = r.actual_TCP_pose_0
    actual_TCP_pose_1_data = r.actual_TCP_pose_1
    actual_TCP_pose_2_data = r.actual_TCP_pose_2
    actual_TCP_pose_3_data = r.actual_TCP_pose_3
    actual_TCP_pose_4_data = r.actual_TCP_pose_4
    actual_TCP_pose_5_data = r.actual_TCP_pose_5


    # 将6个坐标对齐拼接为 setp，并转换为普通浮点数数组
    setp_final = [
        [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        for x, y, z, rx, ry, rz in zip(
            actual_TCP_pose_0_data,
            actual_TCP_pose_1_data,
            actual_TCP_pose_2_data,
            actual_TCP_pose_3_data,
            actual_TCP_pose_4_data,
            actual_TCP_pose_5_data
        )
    ]

    # 打印 setp
    # # print("setp 数据：")
    # for i, pose in enumerate(setp_final):
    #     print(f"样本 {i + 1}: {pose}")
else:
    print("CSV文件中没有 actual_TCP_pose 列。")
#-----------------------------------
# logging.basicConfig(level=logging.INFO)

import serial,time,keyboard
import socket

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
# ser = serial.Serial('/dev/ttyCH341USB0',9600,timeout=1)
# motor_open_list = (0x02,0x00,0x20,0x2f,0x00,0,0xa4) #机械爪松开(具体解释见机械爪用户手册)
# motor_close_list = (0x02,0x01,0x20,0x2f,0x00,0,0xa4)    #机械爪闭合，45字节是角度
motor_open_list = (0x02,0x00,0x20,0x49,0x20,0X00,0xC8) #机械爪松开(具体解释见机械爪用户手册)
motor_close_list = (0x02,0x01,0x20,0x49,0x20,0X00,0xC8)    #机械爪闭合，45字节是角度
import csv

filename = 'gripper.csv'
gripper_list = []

with open(filename, mode='r', newline='') as file:
    reader = csv.reader(file)
    next(reader)  # 跳过第一行（列标题）
    for row in reader:
        gripper_list.append(row[0])  # 将每一行的第一个元素添加到列表



ROBOT_HOST = "192.168.0.201"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

# Setpoints to move the robot to
# setp1 = [-0.12, -0.43, 0.14, 0, 3.11, 0.04]
# setp2 = [-0.12, -0.51, 0.21, 0, 3.11, 0.04]
setp1 = [-0.077, -0.636, 0.341, 2.778, -0.994, 0.047]
setp2 = [-0.553, -0.0188, 0.36397, 1.266, -2.572, -0.049]

setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0


def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list


def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp


# start data synchronization
if not con.send_start():
    sys.exit()

import time

# control loop
move_completed = True
last_pose=[]

current=0
ser.write(motor_open_list)
while current<len(setp_final):
    # receive the current state
    current_pose=setp_final[current]
    current_gripper=gripper_list[current]
    if current==len(setp_final)-1:
        next_pose=current_pose
        next_gripper=current_gripper
    else:
        next_pose=setp_final[current+1]
        next_gripper=gripper_list[current+1]
    state = con.receive()

    if state is None:
        break

    # do something...
    if move_completed and state.output_int_register_0 == 1:
        move_completed = False
        if setp_to_list(setp) == current_pose:
            new_setp = next_pose
            gripper_target=next_gripper
            current+=1
        else:
            new_setp = current_pose
            gripper_target=current_gripper
        list_to_setp(setp, new_setp)
        print("New pose = " + str(new_setp))
        # send new setpoint
        con.send(setp)
        # print(current_gripper)
        # print(gripper_target)
        # print("-------------------")
        # print(type(current_gripper), type(gripper_target))
        if int(current_gripper)==0 and int(gripper_target)==1:
            print("close")
            ser.write(motor_close_list)
        elif int(current_gripper)==1 and int(gripper_target)==0:
            print("open")
            ser.write(motor_open_list)

        time.sleep(0.3)  # 根据机械臂的运动速度调整延时
        watchdog.input_int_register_0 = 1
    elif not move_completed and state.output_int_register_0 == 0:
        # print("Move to confirmed pose = " + str(state.target_q))
        move_completed = True
        watchdog.input_int_register_0 = 0

    # kick watchdog
    con.send(watchdog)
    

con.send_pause()

con.disconnect()
