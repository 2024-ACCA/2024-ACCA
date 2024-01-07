#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import *
import numpy as np

odom = 0
imu = 0

def callback(msg):
    global odom
    _,_,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    # print("Odometry : \n", yaw)
    odom = np.rad2deg(yaw)

def callback_imu(msg):
    global imu
    _,_,yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    # print("IMU : \n", yaw)
    imu = np.rad2deg(yaw)

def main():
    global odom,imu
    rospy.init_node("test", anonymous=True)
    rospy.Subscriber("imu/data", Imu, callback_imu)
    rospy.Subscriber("odometry/navsat", Odometry, callback)
    
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        print("IMU : %.4f, Odom : %.4f" %(imu,odom))
        r.sleep()

if __name__=="__main__":
    main()