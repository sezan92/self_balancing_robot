#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 21 18:08:40 2017

@author: sezan92
"""
import sys,time
import pidcontrol as pid
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
Kp =25
Ki =0.8
Kd =0.1


pubx = pid.PID_Controller(Kp,Ki,Kd)
Yaw_Topic = "/yaw"
cmd_vel = "/cmd_vel"
Imu_topic = "/imu"
Kp_topic = "/Kp"
Ki_topic ="/Ki"
Kd_topic = "/Kd"

class SelfBalance:
    def __init__(self):
        self.pub = rospy.Publisher(cmd_vel,Twist,queue_size =1)
        self.subscriber = rospy.Subscriber(Imu_topic,Imu,self.callback)
        self.subscriber2 = rospy.Subscriber(Kp_topic,Float32,self.callback_Kp)
        self.subscriber3 = rospy.Subscriber(Ki_topic,Float32,self.callback_Ki)
        self.subscriber4 = rospy.Subscriber(Kd_topic,Float32,self.callback_Kd)
        self.pub1 = rospy.Publisher(Yaw_Topic,Float32,queue_size =1)
        self.xvelMin=-.01
        self.xvelMax =0
        self.yMin = -0.01
        self.yMax = -0.001
        self.yPrev =0
        self.delY = 0
        self.Kp = 25
        self.Ki = 0.8
        self.Kd = 0.1
        self.pubx = pid.PID_Controller(self.Kp,self.Ki,self.Kd)
    def callback(self,data):
        setPoint = 0
        y = data.orientation.y
        self.delY = y-self.yPrev
        if self.delY>self.yMax:
            self.yMax = self.delY
        elif self.delY<self.yMin:
            self.yMin = self.delY
        vel = Twist()
        xvel = -self.pubx.getCorrection(setPoint,y)
        if xvel>self.xvelMax:
            self.xvelMax=xvel
        elif xvel<self.xvelMin:
            self.xvelMin = xvel
        if xvel >26:
            xvel =26
        elif xvel<-26:
            xvel =-26
        vel.linear.x = xvel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x =0
        vel.angular.y = 0
        vel.angular.z = 0

        self.pub.publish(vel)
        self.yPrev = y
        self.pub1.publish(y)
        #print "Max vel " + str(self.xvelMax) + " & Min vel " + str(self.xvelMin) + " Max delY " + str(self.yMax*180/3.1416) +" & Min delY" + str(self.yMin*180/3.1416)
    def callback_Kp(self,data):
        self.Kp = data.data
        self.pubx = pid.PID_Controller(self.Kp,self.Ki,self.Kd)
    def callback_Ki(self,data):
        self.Ki = data.data
        self.pubx = pid.PID_Controller(self.Kp,self.Ki,self.Kd)
    
    def callback_Kd(self,data):
        self.Kd = data.data
        self.pubx = pid.PID_Controller(self.Kp,self.Ki,self.Kd)
        

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('SelfBalance', anonymous=True)
    ic = SelfBalance()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS "
    

if __name__ == '__main__':
    main(sys.argv)
