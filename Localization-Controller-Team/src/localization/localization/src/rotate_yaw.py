#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import *
import math as m
import numpy as np

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


class Roatate:
    def __init__(self):
        rospy.init_node("rotate_yaw",anonymous=True)
        self.delta_yaw = rospy.get_param("delta_yaw",default=0.0)
        # rospy.Subscriber("imu/data", Imu, self.callback)
        rospy.Subscriber("ublox_gps/fix_velocity",TwistWithCovarianceStamped,self.callback_gps_vel)
        rospy.Subscriber("odometry/navsat",Odometry,self.callback_odom)
        self.pub = rospy.Publisher("imu/rotated", Imu, queue_size=1)
        self.delta = 0.
        self.gps_yaw = 0.
        self.v = 0.
        self.odom_yaw = 0.
        self.delta = 0.

    def callback_gps_vel(self,msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.v = m.sqrt(vx**2+vy**2)
        self.gps_yaw = m.atan2(vy,vx)

    def callback_odom(self,msg):
        _,_,self.odom_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        delta = self.gps_yaw - self.odom_yaw
        
        if abs(delta) > m.radians(0.0) and abs(delta) < m.radians(30) and self.v > 0.01:
            self.delta = delta
        yaw_prev = self.odom_yaw
        yaw = self.odom_yaw + self.delta_yaw + self.delta
        x,y,z,w = quaternion_from_euler(0, 0, yaw)
        print("%.4f   %.4f" %(m.degrees(self.odom_yaw),m.degrees(yaw)))
        data = Imu()
        data.orientation.x = x
        data.orientation.y = y
        data.orientation.z = z
        data.orientation.w = w
        scale = 10 ** m.degrees(delta)
        data.orientation_covariance = [0.025000000000000005*scale, 0.0, 0.0, 0.0, 0.025000000000000005*scale, 0.0, 0.0, 0.0, 0.025000000000000005*scale]

        self.pub.publish(data)

    # def callback(self,msg):
    #     data = Imu()
    #     _,_,yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    #     delta = self.gps_yaw - yaw
    #     if abs(delta) > m.radians(8) and abs(delta) < m.radians(30) and self.v > 0.3:
    #         self.delta = delta
    #     yaw_prev = yaw
    #     yaw = yaw + self.delta_yaw + self.delta
    #     x,y,z,w = quaternion_from_euler(0, 0, yaw)
    #     print("%.4f   %.4f" %(m.degrees(yaw_prev),m.degrees(yaw)))
    #     data = msg
    #     data.orientation.x = x
    #     data.orientation.y = y
    #     data.orientation.z = z
    #     data.orientation.w = w
    #     scale = 10 ** m.degrees(delta)
    #     data.orientation_covariance = [0.025000000000000005*scale, 0.0, 0.0, 0.0, 0.025000000000000005*scale, 0.0, 0.0, 0.0, 0.025000000000000005*scale]

    #     self.pub.publish(data)

def main():
    r = Roatate()
    rospy.spin()

if __name__=="__main__":
    main()
