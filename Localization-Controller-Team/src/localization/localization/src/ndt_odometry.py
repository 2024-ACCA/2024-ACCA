#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from localization.msg import Measurement
from tf.transformations import *
import numpy as np
import math as m

class NDT:
    def __init__(self,pose_topic,velocity_topic,pub):
        rospy.Subscriber(pose_topic,PoseWithCovarianceStamped,self.callback_pose)  #NDT Scan Matching must be fixed.(Have to make Covariance -> Hessian Matrix)
        rospy.Subscriber(velocity_topic,Float32,self.callback_vel)
        self.pub = rospy.Publisher(pub,Measurement,queue_size=10)

        self.x = 0.
        self.y = 0.
        self.yaw = 0.
        self.init_yaw = None
        self.first_point = None
        self.v = 0.
        self.cov_ndt = None

    def callback_pose(self,msg):
        current_point = Point(msg.pose.pose.position.x,msg.pose.pose.positon.y,0.)
        if self.first_point is None:
            self.first_point = current_point
        self.x = current_point.x - self.first_point.x
        self.y = current_point.y - self.first_point.y
        _,_,yaw = euler_from_quaternion(msg.pose.pose.orientation)
        if self.init_yaw is None:
            self.init_yaw = yaw
        self.yaw = yaw - self.init_yaw
        self.cov_ndt = msg.pose.covariance
    
    def callback_vel(self,msg):
        self.v = msg.data

    def handleData(self):
        state = [self.x,self.y,self.v,self.yaw]
        cov = [self.cov_ndt[0],0.,0.,0.,
               0.,self.cov_ndt[4],0.,0.,
               0.,0.,0.,0.,
               0.,0.,0.,self.cov_ndt[-1]]
        data = Measurement()
        data.state = state
        data.covariance = cov
        self.pub.publish(data)

def main():
    rospy.init_node("ndt_odometry",anonymous=True)

    hz = rospy.get_param("ndt_odom_hz",default=20)
    pose_topic = rospy.get_param("pose_topic",default="ndt_pose")
    vel_topic = rospy.get_param("ndt_velocity_topic",default="estimated_vel_mps")
    pub_topic = rospy.get_param("ndt_odom_topic",default="ndt_odometry")

    rate = rospy.Rate(hz)
    ndt = NDT(pose_topic, vel_topic, pub_topic)
    while not rospy.is_shutdown():
        try:
            ndt.handleData()
        except Exception as ex:
            print(ex)
        rate.sleep()

if __name__ == "__main__":
    main()