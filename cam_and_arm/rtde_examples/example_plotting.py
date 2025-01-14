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

# import matplotlib.pyplot as plt

import sys

sys.path.append("..")
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
    setp = [
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


    print("setp 数据：")
    for i, pose in enumerate(setp):
        print(f"样本 {i + 1}: {pose}")
else:
    print("CSV文件中没有 actual_TCP_pose 列。")


# # plot
# plt.plot(r.timestamp, r.target_q_1)
# plt.show()
