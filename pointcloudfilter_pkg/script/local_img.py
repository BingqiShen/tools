import cv2
import rospy
from nav_msgs.msg import Odometry

img = cv2.imread("/home/mars-nuc/ws/map.png")
print(img.shape)

def poseCallback(msg):
    v = round(741 - 20 * msg.pose.pose.position.x)
    u = round(1165 + 20 * msg.pose.pose.position.y)
    print('u: {} v: {}'.format(u,v))
    cropped = img[v-400:v+400, u-400:u+400]
    # cropped = img[300:1100, 700:1500]
    cv2.imshow('local_img', cropped)
    cv2.waitKey(2)
    

def pose_subscriber():
	# ROS节点初始化
    rospy.init_node('pose_subscriber', anonymous=True)
 
	# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    rospy.Subscriber("/base2map", Odometry, poseCallback)
 
	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    pose_subscriber()
