#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import *
import math as m

class Roatate:
    def __init__(self):
        rospy.init_node("rotate_yaw_o",anonymous=True)
        self.delta_yaw = rospy.get_param("delta_yaw",default=50)
        rospy.Subscriber("imu/data", Imu, self.callback)
        self.pub = rospy.Publisher("imu/rotated_o", Imu, queue_size=1)

    def callback(self,msg):
        _,_,yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        yaw_prev = yaw
        yaw = yaw + m.pi/2*(self.delta_yaw/90)
        x,y,z,w = quaternion_from_euler(0, 0, yaw)
        print("%.4f   %.4f" %(m.degrees(yaw_prev),m.degrees(yaw)))
        data = msg
        data.orientation.x = x
        data.orientation.y = y
        data.orientation.z = z
        data.orientation.w = w

        self.pub.publish(data)

def main():
    r = Roatate()
    rospy.spin()

if __name__=="__main__":
    main()
