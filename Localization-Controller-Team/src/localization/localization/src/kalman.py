#!/usr/bin/env python3

import rospy
from localization.msg import Measurement
from erp42_msg.msg import SerialFeedBack
from tf.transformations import *
from nav_msgs.msg import Odometry
import numpy as np
import math as m

class Kalman:
    def __init__(self):
        rospy.Subscriber("erp42_feedback",SerialFeedBack,self.callback)

        self.x = np.array([
            0, 0, 0, 0
        ])

        self.P = np.array([
            [1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.]
        ])

        self.Q = np.array([
            [0.5, 0., 0., 0.],
            [0., 0.5, 0., 0.],
            [0., 0., 0.01, 0.],
            [0., 0., 0., 0.01]
        ])

        self.cmd = [0,0,0]

    def callback(self,msg):
        self.cmd = [msg.speed, msg.steer, msg.brake]
    
    def inv(self, matrix):
        return np.linalg.inv(matrix)

    def predict(self,x,P,dt):
        A = np.array([
            [1, 0, m.cos(x[3]) * dt, 0],
            [0, 1, m.sin(x[3]) * dt, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        u_k = np.array(
            [0., 0., 0., (self.cmd.data[0] / 1.040) * m.tan(-m.degrees(self.cmd.data[1])) * dt])
        x_k = np.dot(A, x) + u_k
        P_k = np.dot(np.dot(A, P), A.T) + self.Q
        
        return x_k, P_k

    def measurement_update(self,x_k,P_k,z,R):
        H = np.array([[1.,0.,0.,0.],
                      [0.,1.,0.,0.],
                      [0.,0.,1.,0.],
                      [0.,0.,0.,1.]])
        K = np.dot(np.dot(P_k,H.T),self.inv(np.dot(np.dot(H,P_k),H.T)+R))
        x_est = x_k + np.dot(K,(z-np.dot(H,x_k)))
        P_est = P_k - np.dot(K,np.dot(H,P_k))

        return x_est,P_est

    def filter(self,gps,imu,ndt,dt):
        x_pred, P_pred = self.predict(self.x, self.P, dt)
        x_gps, P_gps = self.measurement_update(x_pred, P_pred, gps.state, gps.cov)
        x_imu, P_imu = self.measurement_update(x_gps, P_gps, imu.state, imu.cov)
        x_est, P_est = self.measurement_update(x_imu, P_imu, ndt.state, ndt.cov)
        self.x = x_est
        self.P = P_est

class Odom:
    def __init__(self,topic):
        rospy.Subscriber(topic, Measurement, self.callback)
        self.state = None
        self.cov = None

    def callback(self,msg):
        self.state = np.array(msg.state)
        self.cov = np.array([msg.covariance[0],0.,0.,0.],
                            [0.,msg.covariance[5],0.,0.],
                            [0.,0.,msg.covariance[10],0.],
                            [0.,0.,0.,msg.covariance[-1]])

def main():
    rospy.init_node("kalman",anonymous=True)
    
    gps_topic = rospy.get_param("gps_odom_topic")
    imu_topic = rospy.get_param("imu_odom_topic")
    ndt_topic = rospy.get_param("ndt_odom_topic")

    hz_gps = rospy.get_param("gps_odom_hz")
    hz_imu = rospy.get_param("imu_odom_hz")
    hz_ndt = rospy.get_param("ndt_odom_hz")

    hz = min([hz_gps,hz_imu,hz_ndt])

    kf = Kalman()
    gps = Odom("gps_odometry")
    imu = Odom("imu_odometry")
    ndt = Odom("ndt_odometry")

    rate = rospy.Rate(hz)
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        try:
            dt = current_time - last_time
            kf.filter(gps,imu,ndt,dt)
            last_time = current_time
        except Exception as ex:
            print(ex)
        
        rate.sleep()

if __name__ == "__main__":
    main()