#!/usr/bin/env python
# -*- coding: utf-8 -*
import sys, time
# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
import rosbag
import cv2
from cv_bridge import CvBridge
import glob
from sensor_msgs.msg import Imu
import math
  

# invert bag
bag_rd = rosbag.Bag('/media/nuc/thinking/2022-05-11-13-53-36.bag', "r")
bag_data = bag_rd.read_messages()
bag_wt = rosbag.Bag('/media/nuc/thinking/2022-05-11-13-53-36_equalized.bag', 'w')
window_name = 'Image'

bridge = CvBridge()

for topic, msg, t in bag_data:
    if topic == "/camera1/camera1_resize":
        print('/camera1/camera1_resize.header.stamp: {}'.format(t))
        
        image_color = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_gray = cv2.cvtColor(image_color, cv2.COLOR_BGR2GRAY)
        image_equalized = cv2.equalizeHist(image_gray)
        
        equalized_msg = bridge.cv2_to_imgmsg(image_equalized, encoding="mono8")
        equalized_msg.header = msg.header

        bag_wt.write('/camera1/camera1_resize', equalized_msg, equalized_msg.header.stamp)
    
    elif topic == "/camera2/camera2_resize":
        image_color = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_gray = cv2.cvtColor(image_color, cv2.COLOR_BGR2GRAY)
        image_equalized = cv2.equalizeHist(image_gray)
        
        equalized_msg = bridge.cv2_to_imgmsg(image_equalized, encoding="mono8")
        equalized_msg.header = msg.header

        bag_wt.write('/camera2/camera2_resize', equalized_msg, equalized_msg.header.stamp)
    
    elif topic == "/camera3/camera3_resize":
        image_color = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_gray = cv2.cvtColor(image_color, cv2.COLOR_BGR2GRAY)
        image_equalized = cv2.equalizeHist(image_gray)
        
        equalized_msg = bridge.cv2_to_imgmsg(image_equalized, encoding="mono8")
        equalized_msg.header = msg.header

        bag_wt.write('/camera3/camera3_resize', equalized_msg, equalized_msg.header.stamp)

    elif topic == "/hesai/pandar":
        lidar_msg = msg
        bag_wt.write('/hesai/pandar', lidar_msg, lidar_msg.header.stamp)
    
    elif topic == "/imu_data":
        imu_msg = msg
        bag_wt.write('/imu_data', imu_msg, imu_msg.header.stamp)


bag_wt.close()



