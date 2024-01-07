#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, Float32
from tf.transformations import *
from pyproj import *
import math as m



class PathViewer:
    def __init__(self):
        rospy.Subscriber("ublox_gps/fix", NavSatFix,self.callback)
        rospy.Subscriber("target_idx", Int32, self.callback_idx)
        self.pub_path = rospy.Publisher("path/global", PoseArray, queue_size=10)
        self.pub_speed = rospy.Publisher("target_speed", Float32, queue_size=10)
        
        self.poses = []
        self.speeds = []

        self.gps = Proj(init="epsg:4326")   # lat, lon
        self.tm = Proj(init="epsg:2097")    # m

        self.gps_datum = None
        self.index = 0

    def callback(self,msg):
        if self.gps_datum is None:
            self.gps_datum = [msg.latitude, msg.longitude]
            print(self.gps_datum)

    def file_opener(self,file):    
        f = open(file, "r")
        
        lines = f.readlines()
        
        poses = []
        speeds = []
        num = 0
        x_o, y_o = transform(p1=self.gps, p2=self.tm, x=self.gps_datum[1], y=self.gps_datum[0])
        
        time = rospy.Time.now()
        for line in lines:
            pose = Pose()
            
            l = line.rstrip("\n")
            l_split = l.split(", ")
            x,y,yaw = float(l_split[0]),float(l_split[1]),float(l_split[2]) 
            # x, y = transform(p1=self.gps, p2=self.tm, x=lon, y=lat)

            pose.position.x = x-x_o
            print(x_o, y_o)
            pose.position.y = y-y_o
            pose.position.z = 0

            qx,qy,qz,qw = quaternion_from_euler(0, 0, m.radians(yaw))

            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            poses.append(pose)
            speeds.append(float(l_split[3]))
            num +=1
        f.close()
        return poses, speeds

    def publish_path(self,frame_id,file):
        # self.gps_datum = [37.23918087249071, 126.77312842007436]
        if not self.gps_datum is None:
            # print("test")
            if len(self.poses) == 0:
                self.poses, self.speeds = self.file_opener(file)

            data = PoseArray()
            data.header.stamp = rospy.Time.now()
            data.header.frame_id = frame_id
            data.header.seq += 1
            data.poses = self.poses
            self.pub_path.publish(data)

    def publish_target_speed(self,target_idx):
        data = Float32()
        data.data = self.speeds[target_idx]
        self.pub_speed.publish(data)

    def callback_idx(self,msg):
        self.index = msg.data

def main():
    rospy.init_node("path_opener",anonymous=True)

    p = PathViewer()
    file_path = rospy.get_param("file_path",default="/home/acca/Downloads/gpt_820_converted(1).txt")

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        # print("alive")
        try:
            p.publish_path("odom",file=file_path)
            p.publish_target_speed(p.index)
            # print(p.gps_datum)
        except Exception as ex:
            print(ex)
        rate.sleep()

if __name__=="__main__":
    main()

