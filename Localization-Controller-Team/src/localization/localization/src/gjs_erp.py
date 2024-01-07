#!/usr/bin/env python3

import rospy
from erp42_msgs.msg import SerialFeedBack
from nav_msgs.msg import Odometry
import math as m    



class ekf_v:
    def __init__(self):
        rospy.Subscriber("/odometry/navsat", Odometry,self.callback)

    def callback(self, msg):
        self.v = m.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

class erp_v:
    def __init__(self):
        rospy.Subscriber("/erp42_feedback",SerialFeedBack,self.callback)

    def callback(self, msg):
        self.v = msg.speed



def main():
    rospy.init_node("compare_v",anonymous=True)

    k = ekf_v()
    p = erp_v()

    rate = rospy.Rate(8)
    
    while not rospy.is_shutdown():
        try:
            print("ekf_v:{}, erp_v:{}".format(k.v, p.v))
        except Exception as ex:
            print(ex)
        rate.sleep()


if __name__=="__main__":
    main()
