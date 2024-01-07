#!/usr/bin/env python3

import rospy
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
from tf.transformations import *

class ErpTwist:
    def __init__(self):
        rospy.init_node("erp_twist", anonymous=True)
        rospy.Subscriber("erp42_feedback", SerialFeedBack, self.callback_erp)
        rospy.Subscriber("imu/data", Imu, self.callback_imu)
        self.pub = rospy.Publisher("erp42/twist", TwistWithCovarianceStamped,queue_size=10)
        self.yaw = None
        self.header = Header()

    def callback_erp(self,msg):
        if not self.yaw is None:
            yaw = self.yaw
            header = self.header
            gear = msg.Gear
            if gear == 2:
                v = msg.speed
            else:
                v = (-1) * msg.speed
            self.publishTwist(v,yaw,header)

    def callback_imu(self,msg):
        quarternion = msg.orientation
        _,_,self.yaw = euler_from_quaternion([quarternion.x,quarternion.y,quarternion.z,quarternion.w])

    def publishTwist(self,v,yaw,header):
        data = TwistWithCovarianceStamped()

        data.header = header

        data.twist.twist.linear.x = v * np.cos(yaw)
        data.twist.twist.linear.y = v * np.sin(yaw)
        data.twist.twist.linear.z = 0

        data.twist.twist.angular.x = 0
        data.twist.twist.angular.y = 0
        data.twist.twist.angular.z = 0

        data.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
        self.pub.publish(data)

def main():
    et = ErpTwist()
    rospy.spin()

if __name__=="__main__":
    main()