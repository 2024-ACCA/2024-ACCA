#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Int32, String
from erp42_msgs.msg import SerialFeedBack
from stanley import Stanley
from erp42_control.msg import ControlMessage
from tf.transformations import *
import numpy as np
import math as m
from geometry_msgs.msg import PoseArray

class PathHandler:
    def __init__(self,path_topic):
        rospy.Subscriber(path_topic,PoseArray,self.callback_path)

        self.cx = []
        self.cy = []
        self.cyaw = []

    def callback_path(self,msg):
        self.cx, self.cy, self.cyaw = self.update_path(msg)

    def update_path(self, data):
        cx = []
        cy = []
        cyaw = []
        for p in data.poses:
            cx.append(p.position.x)
            cy.append(p.position.y)
            _,_,yaw = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]) 
            cyaw.append(yaw)    
        return cx, cy, cyaw 

class State:
    def __init__(self, odom_topic):
        rospy.Subscriber(odom_topic, Odometry,self.callback)
        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s

    def callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _,_,self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.v = m.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

class PID(object):
    def __init__(self):
        self.p_gain = rospy.get_param("/stanley_controller/p_gain", 2.6)
        self.i_gain = rospy.get_param("/stanley_controller/i_gain", 0.5)
        self.d_gain = rospy.get_param("/stanley_controller/d_gain", 1.0)

        self.p_err = 0.0
        self.i_err = 0.0
        self.d_err = 0.0

        self.current = rospy.Time.now()
        self.last = rospy.Time.now()

    def PIDControl(self, speed, desired_value):
        self.current = rospy.Time.now()
        
        dt = (self.current - self.last).to_sec()
        
        err = desired_value - speed

        self.d_err = (err - self.p_err) / dt 
        self.p_err = err
        self.i_err += self.p_err * dt * self.i_gain * (0.0 if speed == 0 else 1.0)

        self.last = self.current

        speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err) + (self.d_gain * self.d_err)
        
        return int(np.clip(speed, 0, 20)) 
    
class SpeedSupporter:
    def __init__(self):
        self.he_gain = rospy.get_param("/speed_supporter/he_gain", 30.0)
        self.ce_gain = rospy.get_param("/speed_supporter/ce_gain", 20.0)

        self.he_thr = rospy.get_param("/speed_supporter/he_thr",0.01)
        self.ce_thr = rospy.get_param("/speed_supporter/ce_thr",0.08)

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self,value,hdr,ctr,min_value,max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        # err = ctr
        res = np.clip(value + err, min_value, max_value)

        return res

class Drive:
    def __init__(self,state,path,target_speed):
        rospy.Subscriber(target_speed,Float32,self.callback_speed)
        rospy.Subscriber("erp42_feedback",SerialFeedBack,self.callback_erp)
        rospy.Subscriber("path_type",String,self.callback_path_type)
        self.pub = rospy.Publisher("cmd_msg",ControlMessage,queue_size=10)
        self.pub_idx = rospy.Publisher("target_idx", Int32, queue_size=10)

        self.st = Stanley()
        self.pid = PID()
        self.ss = SpeedSupporter()
        self.path = path
        self.state = state
        self.last_target_idx_global = 0
        self.path_speed = 6
        self.current_speed = 0.
        self.path_type = "global"
        self.count = 0

        self.max_steer = 28 

    def publish_cmd(self):
        path_type = self.path_type
        if path_type == "global":
            last_target_idx = self.last_target_idx_global
        else:
            last_target_idx = 0

        steer, target_idx, hdr, ctr = self.st.stanley_control(self.state, self.path.cx, self.path.cy, self.path.cyaw, last_target_idx)
        
        if path_type == "global":
            self.last_target_idx_global = target_idx
            path_speed = self.path_speed
        else:
            path_speed = 4

        adaptedspeed = self.ss.adaptSpeed(path_speed,hdr,ctr,min_value=4,max_value=20)
        steer = np.clip(steer,m.radians((-1)*self.max_steer),m.radians(self.max_steer))
        speed = self.pid.PIDControl(self.current_speed*3.6, adaptedspeed)

        msg = ControlMessage()
        msg.Speed = speed*10
        msg.Steer = int(m.degrees((-1)*steer))
        msg.Gear = 2
        msg.brake = 0

        self.count += 1

        rospy.loginfo(self.state.v)
        rospy.loginfo("Adapted Speed : %2.4f" %(adaptedspeed))
        rospy.loginfo("Speed : %2.4f Steer : %2.4f" %(msg.Speed,msg.Steer))
        # print(self.count)

        self.pub.publish(msg)
        self.pub_idx.publish(target_idx)

    def callback_speed(self,msg):
        self.path_speed = msg.data

    def callback_erp(self,msg):
        direction = 1.0 if msg.Gear == 2 else -1.0
        self.current_speed = msg.speed * direction

    def callback_path_type(self,msg):
        self.path_type = msg.data

def main():
    rospy.init_node("global_driving_node", anonymous=True)
    state = State("odometry/final")
    path_tracking = PathHandler("path/global")
    d = Drive(state,path_tracking,"target_speed")
    
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            d.publish_cmd()
        except Exception as ex:
            print(ex)
        rate.sleep()

if __name__=="__main__":
    main()
