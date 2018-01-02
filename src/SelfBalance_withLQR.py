#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 21 18:08:40 2017

@author: sezan92
"""
import sys,time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from control import lqr
import numpy as np
from std_msgs.msg import Float32

A = np.array([[0,1],[47.77,0]])
B = np.array([[  0.  ],
       [ 10.52]])
Q = np.array([[ 100,   0],
       [  0, 1000]])
R = 0.0001
K,S,e = lqr(A,B,Q,R)

cmd_vel = "/cmd_vel"
Imu_topic = "/imu"
Yaw_Topic = "/yaw"
q_topic = "/Q"
r_topic ="/R"
class SelfBalanceLQR:
    def __init__(self):
        self.pub = rospy.Publisher(cmd_vel,Twist,queue_size =1)
        self.subscriber = rospy.Subscriber(Imu_topic,Imu,self.callback)
        self.subscriber2 = rospy.Subscriber(q_topic,Float32,self.callback_q)
        self.subscriber3= rospy.Subscriber(r_topic,Float32,self.callback_r)
        self.pub1 = rospy.Publisher(Yaw_Topic,Float32,queue_size =1)
        self.xvelMin=-.01
        self.xvelMax =0
        self.yMin = -0.01
        self.yMax = 0
        self.y_ = 0
        self.Q = np.array([[ 100,   0],[  0, 1000]])
        self.R = 0.0001
        #xvel = xvel1.output['velocity']
        self.K,self.S,self.e = lqr(A,B,self.Q,self.R)
    def callback(self,data):
        self.K,self.S,self.e = lqr(A,B,self.Q,self.R)
        y = data.orientation.y*180/3.1416
        if y>self.yMax:
            self.yMax = y
        elif y<self.yMin:
            self.yMin =y
        vel = Twist()
        diff_yaw = y-self.y_
        np_x = np.array([[y],[diff_yaw]])

        xvel = self.K.dot(np_x)[0,0]
       
        if xvel>self.xvelMax:
            self.xvelMax=xvel
        elif xvel<self.xvelMin:
            self.xvelMin = xvel
        vel.linear.x = xvel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x =0
        vel.angular.y = 0
        vel.angular.z = 0
        self.pub.publish(vel)
        #print "Max vel " + str(self.xvelMax) + " & Min vel " + str(self.xvelMin) + " Max y " + str(self.yMax*180/3.1416) +" & Min y" + str(self.yMin*180/3.1416)
        #print "Velocity "+ str(xvel)+ " & yaw " + str(y)
        self.y_ = y
        self.pub1.publish(data.orientation.y)
    def callback_q(self,data):
        q = data.data
        self.Q = np.array([[ q,   0],[  0, 10*q]])
        
        
    def callback_r(self,data):
        r = data.data
        self.R = r
        
def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('SelfBalance', anonymous=True)
    ic = SelfBalanceLQR()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS "
    

if __name__ == '__main__':
    main(sys.argv)