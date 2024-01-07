#!/usr/bin/env python3

import rospy
from tf.transformations import *
from erp42_control.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class StateMaker:
    def __init__(self):
        rospy.init_node("state_publisher",anonymous=True)
        rospy.Subscriber("odometry/kalman", Odometry, callback = self.callback_kalman)
        rospy.Subscriber("ndt_pose",PoseStamped, callback = self.callback_ndt)
        self.state = State()
        self.state_kalman = State()
        self.state_ndt = State()

    def callback_kalman(self,msg):
        self.state_kalman.x = msg.pose.pose.position.x
        self.state_kalman.y = msg.pose.pose.position.y
        _,_,yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.state_kalman.yaw = yaw

    def callback_ndt(self,msg):
        self.state_ndt.x = msg.pose.position.x
        self.state_ndt.y = msg.pose.position.y
        _,_,yaw = euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        self.state_ndt.yaw = yaw



def main():
    s = StateMaker()
    
if __name__=="__main__":
    main()