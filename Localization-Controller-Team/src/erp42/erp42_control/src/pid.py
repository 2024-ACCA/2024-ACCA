#!/usr/bin/env python

import os
import sys
import rospy
import rospkg
import math as m
import numpy as np
from state import OdomState
from std_msgs.msg import *
from erp42_msgs.msg import *
from erp42_control.msg import *
import matplotlib.pyplot as plt
from time import time


class PID(object):
    def __init__(self):
        self.p_gain = 2.7
        self.i_gain = 0.7
        self.d_gain = 0.35

        self.p_err = 0.0
        self.i_err = 0.0
        self.d_err = 0.0
        
        self.speed = 0

        self.current = rospy.Time.now()
        self.last = rospy.Time.now()

        rospy.Subscriber(
            "/erp42_feedback", SerialFeedBack, callback=self.erpCallback
        )

        rospy.Subscriber("p_gain", Float32, callback=self.p_callback)
        rospy.Subscriber("i_gain", Float32, callback=self.i_callback)
        rospy.Subscriber("d_gain", Float32, callback=self.d_callback)


    def erpCallback(self, msg):
        self.speed = mps2kph(msg.speed)

    def p_callback(self, msg):
        self.p_gain = msg.data

    def i_callback(self, msg):
        self.i_gain = msg.data

    def d_callback(self, msg):
        self.d_gain = msg.data

    def PIDControl(self, desired_value):
        self.current = rospy.Time.now()
        
        dt = (self.current - self.last).to_sec()
        
        err = desired_value - self.speed

        self.d_err = (err - self.p_err) / dt
        self.p_err = err
        self.i_err += self.p_err * dt * self.i_gain * (0.0 if self.speed == 0 else 1.0)

        self.last = self.current

        speed = self.speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err) + (self.d_gain * self.d_err)
        
        final_speed = round(np.clip(speed, 0, 25),1)
        
        return final_speed*10


def kph2mps(value):
    return value * 0.277778


def mps2kph(value):
    return value * 3.6


def erpCallback(msg):
    global speed
    speed = msg.speed


if __name__ == "__main__":
    rospy.init_node("pid_tuner test")

    speed = 0.

    erp_sub = rospy.Subscriber(
        "/erp42_feedback", SerialFeedBack, callback=erpCallback
    )
    cmd_pub = rospy.Publisher(
        "/cmd_msg", ControlMessage, queue_size=1)

    pid = PID()

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    init_time = time()

    r = rospy.Rate(50)
    v1 = []      #input_speed
    dt1 = []     #current_time
    v2 = []      #speed

    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        dt1.append(time() - init_time)

        input_speed = pid.PIDControl(desired_value=4)
        v1.append(float(input_speed)/10.)
        last_time = current_time
        v2.append(float(pid.speed)) 

        print(pid.speed, float(input_speed)/10.)

        cmd_pub.publish(
            ControlMessage(
                0, 0, 2, (int(input_speed)), -1.7, 0, 0
            )
        )

        r.sleep()

plt.scatter(dt1, v1, c = 'r', s = 0.1) 
plt.scatter(dt1, v2, c = 'b', s = 0.1) 
plt.plot(dt1, [4]*len(dt1), c = 'g')
	
plt.grid(True)
plt.xlabel('time')
plt.axis('equal')
plt.show()