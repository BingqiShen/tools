#!/usr/bin/env python
# -*- coding: utf-8 -*
from cmath import asin
import sys, time
import os

# numpy and scipy
import numpy as np


# Ros libraries
import roslib
import rospy

# Ros Messages
import rosbag
import cv2
from cv_bridge import CvBridge
import glob
import math
from visual_msgs.msg import VioState


# new state
state = []
i = 0
time_index = 0
img_time_list = []

def time_select(time_list, time_):
    # global time_index
    time_index = 0
    while (time_index < len(time_list)-1):
        if (abs(time_.to_sec()- time_list[time_index].to_sec()) > 0.001):
            time_index = time_index +1
            continue
        print("time is %9f, time list is %9f", (time_.to_sec(), time_list[time_index].to_sec())) 
        break

    return time_list[time_index]


def decifunc(string_a, n):
    a, b, c = string_a.partition('.')
    cc = c[:n]
    if int(c[n]) >= 5:
        print(string_a)
        ccc = float(cc) + 1
        print(ccc)
    else:
        ccc = float(cc)

    ccc = int(ccc)
    string_after = a + str(ccc)

    return int(a), ccc


with open("/home/jz/jz_ws/test_ws/src/visual_msgs/script/openvins_odom.txt", "r") as f:
    for line in f.readlines():
        line = line.strip('\n')
        data = line.split()

        item = VioState()
        sec, nsec = decifunc(data[0], 9)
        item.header.stamp = rospy.Time(sec, nsec)

        item.T_M_I.rotation.x = float(data[1])
        item.T_M_I.rotation.y = float(data[2])
        item.T_M_I.rotation.z = float(data[3])
        item.T_M_I.rotation.w = float(data[4])
        item.T_M_I.translation.x = float(data[5])
        item.T_M_I.translation.y = float(data[6])
        item.T_M_I.translation.z = float(data[7])

        # rx = math.atan2(2*(float(data[4])*float(data[1])+float(data[2])*float(data[3])), 1-2*(float(data[1])**2+float(data[1])**2))
        # ry = math.asin(2*(float(data[4])*float(data[2]) - float(data[1])*float(data[3])))
        # rz = math.atan2(2*(float(data[4])*float(data[3])+float(data[1])*float(data[2])), 1-2*(float(data[2])**2+float(data[3])**2))


        # if i != 0:
        #     delta_t = float(data[0]) - last_t
        #     item.v_M_I = [(item.T_M_I.translation.x - last_x) / delta_t,
        #                     (item.T_M_I.translation.y - last_y) / delta_t,
        #                     (item.T_M_I.translation.z - last_z) / delta_t,
        #                     (rx - last_rx) / delta_t,
        #                     (ry - last_ry) / delta_t,
        #                     (rz - last_rz) / delta_t,]
        
        # last_x = item.T_M_I.translation.x
        # last_y = item.T_M_I.translation.y
        # last_z = item.T_M_I.translation.z
        # last_rx = rx
        # last_ry = ry
        # last_rz = rz
        # last_t = float(data[0])
        item.v_M_I = [float(data[8]), float(data[9]), float(data[10])]

        item.Gyb = [float(data[11]), float(data[12]), float(data[13])]
        item.Acb = [float(data[14]), float(data[15]), float(data[16])]

        state.append(item)
        i = i + 1
        print(i)

print(len(state))

# # original bag
# bag_rd = rosbag.Bag('/media/jz/5b3f1040-e4bd-4eee-8340-b1e1fbc574ba/home/jz/hfz_calibration/test.bag', "r")
# bag_data = bag_rd.read_messages()
bag_wt = rosbag.Bag('/media/jz/5b3f1040-e4bd-4eee-8340-b1e1fbc574ba/home/jz/hfz_calibration/openvins_viostate.bag', 'w')
# window_name = 'Image'

# bridge = CvBridge()

# for topic, msg, t in bag_data:
#     if topic == "/left/image_raw":
#         invert_msg = msg
#         msg.header.stamp = t
#         bag_wt.write('/left/image_raw', invert_msg, t)
#     elif topic == "/right/image_raw":
#         imu_msg = msg
#         bag_wt.write('/right/image_raw', imu_msg, t)
#     elif topic == "/stereo_left/image_raw":
#         imu_msg = msg
#         img_time_list.append(t)
#         bag_wt.write('/stereo_left/image_raw', imu_msg, t)
#     elif topic == "/stereo_right/image_raw":
#         imu_msg = msg
#         bag_wt.write('/stereo_right/image_raw', imu_msg, t)
#     elif topic == "/imu_data":
#         imu_msg = msg
#         bag_wt.write('/imu_data', imu_msg, t)


for i in range(len(state)):
    state_msg = state[i]
    # time = time_select(img_time_list, state_msg.header.stamp)
    # state_msg.header.stamp =  time
    bag_wt.write('/vio_state', state_msg, state_msg.header.stamp)

bag_wt.close()



