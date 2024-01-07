#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from tf.transformations import *
from stanley import Stanley
import math as m
import numpy as np

class Roatate:
    def __init__(self):
        self.delta_yaw = rospy.get_param("~delta_yaw",default=0.0)
        rospy.Subscriber("ublox_gps/fix_velocity",TwistWithCovarianceStamped,self.callback_gps_vel)
        rospy.Subscriber("odometry/kalman",Odometry,self.callback_kalman)
        # rospy.Subscriber("path/global", PoseArray, self.callback_path)
        rospy.Subscriber("target_idx", Int32, self.callback_idx)
        rospy.Subscriber("road_type", String, self.callback_shape)
        self.pub = rospy.Publisher("imu/rotated", Imu, queue_size=1)
        self.gps_yaw = 0.
        self.kalman_yaw = 0.
        self.delta = 0.
        self.target_index = 0
        self.v = 0.
        self.index = 0
        self.path_shape ="straight"
        
    def callback_idx(self,msg):
        self.index = msg.data

    def callback_gps_vel(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.v = vx**2 + vy**2
        self.gps_yaw = m.atan2(vy,vx)

    def callback_kalman(self, msg):
        _,_,self.kalman_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

    def callback_shape(self, msg):
        self.path_shape = msg.data   # path_opener 파일에서 [5]를 path_shape(straight or curve 판단)


    def publish_yaw(self):
        msg = Imu()

        delta = self.gps_yaw - self.kalman_yaw

        if abs(delta) > m.radians(6) and self.v > 0.01 and self.path_shape == "straight":
            self.delta = delta

        yaw = self.kalman_yaw + self.delta_yaw + self.delta

        x,y,z,w = quaternion_from_euler(0, 0, yaw)

        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w

        scale = (10 ** m.degrees(delta))
        msg.orientation_covariance = [0.025000000000000005*scale, 0.0, 0.0, 0.0, 0.025000000000000005*scale, 0.0, 0.0, 0.0, 0.025000000000000005*scale]


        self.pub.publish(msg)

def main():
    rospy.init_node("rotate_yaw",anonymous=True)

    r = Roatate()

    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        try:
            r.publish_yaw()
        except Exception as ex:
            print(ex)
        rate.sleep()


if __name__=="__main__":
    main()
