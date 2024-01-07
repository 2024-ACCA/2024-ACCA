#!/usr/bin/env python3

import rospy
import math as m
import numpy as np
from erp42_msgs.msg import State
from std_msgs.msg import Float32, Int32, Bool
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
from stanley import Stanley

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = m.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = m.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = m.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  

class Controller:
    def __init__(self):
        self.max_steer = 20
        self.staley = Stanley()
        self.last_index = 0
        self.obs = False
        self.global_path = [[],[],[]]
        self.local_path = [[],[],[]]

        rospy.Subscriber("car_state",State,self.callback_state)
        rospy.Subscriber("obs_state",Bool, self.callback_obs)
        rospy.Subscriber("realPathnode",MarkerArray,self.callback_local)
        rospy.Subscriber("global_path",PoseArray,self.callback_global)

        self.pub_steer = rospy.Publisher("stanley_steer",Float32,queue_size=1)
        self.pub_index = rospy.Publisher("target_index",Int32,queue_size=1)
    
    def callback_state(self,msg):
        state = State()
        state = msg
        if self.obs == False:
            self.calc_steer(state,self.global_path)
        else:
            self.calc_steer(state,self.local_path)

    def callback_obs(self,msg):
        self.obs = msg.data

    def callback_local(self,msg):
        self.local_path = [[],[],[]]
        for ma in msg.markers:
            _,_,yaw = euler_from_quaternion(ma.pose.orientation.x,ma.pose.orientation.y,ma.pose.orientation.z,ma.pose.orientation.w)
            point = [ma.pose.position.x,ma.pose.position.y,yaw]
            i = 0
            for p in self.local_path:
                p.append(point[i])
                i += 1

    def callback_global(self,msg):
        self.global_path = [[],[],[]]
        for po in msg.poses:
            _,_,yaw = euler_from_quaternion(po.orientation.x,po.orientation.y,po.orientation.z,po.orientation.w)
            point = [po.position.x,po.position.y,yaw]
            i = 0
            for p in self.global_path:
                p.append(point[i])
                i += 1
          
    def calc_steer(self,state,path):
        try:
            delta, target_index = self.staley.stanley_control(state,path[0],path[1],path[2],self.last_index)
            self.last_index = target_index
            delta = np.clip(delta, -m.radians(self.max_steer), m.radians(self.max_steer))
            self.pub_steer(delta)
            self.pub_index(target_index)

        except IndexError as ie:
            rospy.logwarn(ie)
        
        except Exception as ex:
            rospy.logwarn(ex)

def main():
    rospy.init_node("stanley_bs_ts_D",anonymous=True)
    c = Controller()

if __name__ == "__main__":
    main()