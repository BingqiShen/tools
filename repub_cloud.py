#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu


pc_msg_new = PointCloud2()
imu_msg_new = Imu()
#first_flag = True

def PcCallbak(ros_cloud):
    global first_flag, init_time
    pc_msg_new = ros_cloud
    new_time = time0.to_sec() + ros_cloud.header.stamp.to_sec() - init_time
    pc_msg_new.header.stamp =  rospy.Time.from_sec(new_time) 
    pub1.publish(pc_msg_new)

def ImuCallbak(ros_imu):
    global first_flag, init_time
    if first_flag == True:
        init_time = ros_imu.header.stamp.to_sec()
        first_flag = False 
    imu_msg_new = ros_imu
    new_time = time0.to_sec() + ros_imu.header.stamp.to_sec() - init_time
    imu_msg_new.header.stamp = rospy.Time.from_sec(new_time) 
    pub2.publish(imu_msg_new)

if __name__ == '__main__':
    global pub1, pub2 ,time0, init_time, first_flag
    first_flag = True;
    rospy.init_node('repubPointCloud1',anonymous = True)
    time0 = rospy.Time.now()

    sub1 = rospy.Subscriber("/os_cloud_node/points",PointCloud2,PcCallbak)
    sub2 = rospy.Subscriber("/os_cloud_node/imu",Imu,ImuCallbak)
    pub1 = rospy.Publisher('/robot_Z/pointcloud',PointCloud2,queue_size=10)
    pub2 = rospy.Publisher('/robot_Z/imu',Imu,queue_size=10)
    rospy.spin()
