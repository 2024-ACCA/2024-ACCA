#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from localization.msg import Measurement
from pyproj import *
import numpy as np
import math as m

class GPS:
    def __init__(self,topic_fix,topic_velocity,pub):
        rospy.Subscriber(topic_fix,NavSatFix,self.callback_fix)
        rospy.Subscriber(topic_velocity,TwistWithCovarianceStamped,self.callback_velocity)
        self.pub = rospy.Publisher(pub,Measurement,queue_size=10)

        self.gps = Proj(init="epsg:4326")   # lat, lon
        self.tm = Proj(init="epsg:2097")    # m
        
        self.first_point = None

        self.dx = 0.
        self.dy = 0.
        self.v  = 0.
        self.heading = 0.

        self.cov_fix = 0
    
    def callback_fix(self,msg):
        x, y = transform(p1=self.gps, p2=self.tm, x=msg.longitude, y=msg.latitude)
        current_point = Point(x, y, 0.)
        if self.first_point is None:
            self.first_point = current_point
        self.dx = current_point.x - self.first_point.x
        self.dy = current_point.y - self.first_point.y
        self.cov_fix = msg.position_covariance

    def callback_velocity(self,msg):
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        self.v = m.sqrt(v_x**2 + v_y**2)
        if self.v > 0.01:
            self.heading = m.atan2(v_y/v_x)

    def handleData(self):
        state = [self.dx,self.dy,self.v,self.heading]
        cov = [self.cov_fix[0],0.,0.,0.,
               0.,self.cov_fix[4],0.,0.,
               0.,0.,0.01,0.,
               0.,0.,0.,0.1]
        data = Measurement()
        data.state = state
        data.covariance = cov
        self.pub.publish(data)

def main():
    rospy.init_node("gps_odometry",anonymous=True)

    hz = rospy.get_param("gps_odom_hz",default=8)
    fix_topic = rospy.get_param("fix_topic",default="ublox_gps/fix")
    velocity_topic = rospy.get_param("fix_velocity_topic",default="ublox_gps/fix_velocity")
    pub_topic = rospy.get_param("gps_odom_topic",default="gps_odometry")

    gps = GPS(fix_topic,velocity_topic,pub_topic)
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        try:
            gps.handleData()
        except Exception as ex:
            print(ex)
        rate.sleep()

if __name__ == "__main__":
    main()