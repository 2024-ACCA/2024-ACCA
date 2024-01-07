#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from erp42_msg.msg import SerialFeedBack
from tf.transformations import *
import numpy as np
import math as m

class IMUEncoder:
    def __init__(self,imu_topic,encoder_topic,pub):
        rospy.Subscriber(imu_topic,Imu,self.callback_imu)
        rospy.Subscriber(encoder_topic,SerialFeedBack,self.callback_encoder)
        self.pub = rospy.Publisher(pub,Measurement,queue_size=10)
        
        self.yaw = 0.
        self.init_yaw = None
        self.v = 0.
        self.cov = 0.
        self.x = 0.
        self.y = 0.
        self.last_time = rospy.Time.now()

    def callback_imu(self,msg):
        _,_,yaw = euler_from_quaternion(msg.orientation)
        if self.init_yaw is None:
            self.init_yaw = yaw
        self.yaw = yaw - self.init_yaw
        self.cov = msg.orientation_covariance

    def callback_encoder(self,msg):
        v = msg.speed
        direction = 1.0 if msg.Gear == 2 else -1.0
        self.v = msg.speed * direction

    def handleData(self):
        current_time = rospy.Time.now()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.x += dt*self.v*m.cos(self.yaw)
        self.y += dt*self.v*m.sin(self.yaw)

        state = [self.x,self.y,self.v,self.yaw]
        cov = [0.5,0.,0.,0.,
               0.,0.5,0.,0.,
               0.,0.,0.01,0.,
               0.,0.,0.,self.cov[-1]]
        data = Measurement()
        data.state = state
        data.covariance = cov
        self.pub.publish(data)

def main():
    rospy.init_node("imu_odometry",anonymous=True)

    hz = rospy.get_param("imu_odom_hz",default=100)
    imu_topic = rospy.get_param("imu_topic",default="imu/data")
    encoder_topic = rospy.get_param("encoder_topic",default="erp42_feedback")
    pub_topic = rospy.get_param("imu_odom_topic",default="imu_odometry")

    imu = IMUEncoder(imu_topic, encoder_topic, pub_topic)
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        try:
            imu.handleData()
        except Exception as ex:
            print(ex)
        rate.sleep()

if __name__ == "__main__":
    main()