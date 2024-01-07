#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, Float32, String
from tf.transformations import *
from pyproj import *
import math as m



class PathViewer:
    def __init__(self):
        rospy.Subscriber("ublox_gps/fix", NavSatFix,self.callback)
        rospy.Subscriber("target_idx", Int32, self.callback_idx)
        self.pub_path = rospy.Publisher("path/global", PoseArray, queue_size=10)
        self.pub_speed = rospy.Publisher("target_speed", Float32, queue_size=10)
        self.pub_tag = rospy.Publisher("mission_tag",String,queue_size=10)
        self.pub_road_type = rospy.Publisher("road_type",String,queue_size=10)
        
        self.poses = []
        self.speeds = []
        self.tags = []
        self.road_type = []

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
        tags = []
        road_type = []
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
            pose.position.y = y-y_o
            pose.position.z = 0

            qx,qy,qz,qw = quaternion_from_euler(0, 0, m.radians(yaw))

            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            poses.append(pose)
            speeds.append(float(l_split[3]))
            tags.append(l_split[4])
            road_type.append(l_split[5])
            num +=1
        f.close()
        return poses, speeds, tags, road_type

    def publish_path(self,frame_id,file):
        # self.gps_datum = [37.23918087249071, 126.77312842007436]
        if not self.gps_datum is None:
            # print("test")
            if len(self.poses) == 0:
                self.poses, self.speeds, self.tags, self.road_type = self.file_opener(file)
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

    def publish_tag(self,target_idx):
        data = String()
        data.data = self.tags[target_idx]
        self.pub_tag.publish(data)

    def publish_road_type(self,target_idx):
        data = String()
        data.data = self.road_type[target_idx]
        self.pub_road_type.publish(data)

    def callback_idx(self,msg):
        self.index = msg.data

def main():
    rospy.init_node("path_opener",anonymous=True)

    p = PathViewer()
    # file_path = rospy.get_param("file_path",default="/home/acca/Downloads/bonsun_parking_path_0909.txt")
    # file_path = rospy.get_param("file_path",default="/home/acca/Downloads/gpt_821_converted.txt")
    # file_path = rospy.get_param("file_path",default="/home/acca/school_tm_0904.txt")
    # file_path = rospy.get_param("file_path",default="/home/acca/Downloads/bonsun_parking_path_0909 (1).txt")
    # file_path = rospy.get_param("file_path",default="/home/acca/Downloads/school_speed_10.txt")
    file_path = rospy.get_param("file_path",default="/home/acca/Downloads/1010_school_global.txt")
    # file_path = rospy.get_param("file_path",default="/home/acca/Downloads/highway_real_tm.txt")

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            p.publish_path("odom",file=file_path)
            p.publish_target_speed(p.index)
            p.publish_tag(p.index)
            p.publish_road_type(p.index)

        except Exception as ex:
            print(ex)

        rate.sleep()

if __name__=="__main__":
    main()

