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
from SelfBalancingFuzzyP import *
from std_msgs.msg import Float32

Kp =1000
Ki =0.1
Kd =0.1
counter =0

pubx = pid.PID_Controller(Kp,Ki,Kd)
cmd_vel = "/cmd_vel"
Imu_topic = "/ardrone/imu"
Yaw_Topic = "/pitch"
Input = "/Input"
class SelfBalance:
    def __init__(self):
        
        self.subscriber2 = rospy.Subscriber(Input,Float32,self.callback2)
        self.pub = rospy.Publisher(cmd_vel,Twist,queue_size =1)
        self.subscriber = rospy.Subscriber(Imu_topic,Imu,self.callback)
        self.pub1 = rospy.Publisher(Yaw_Topic,Float32,queue_size =1)
        self.xvelMin=-.01
        self.xvelMax =0
        self.yMin = -0.01
        self.yMax = 0
        self.y_ = 0
        self.setPoint=0
    def callback2(self,data):
        self.setPoint=data.data
    def callback(self,data):
        global counter
        stamp = data.header.stamp
        time = stamp.secs+stamp.nsecs*1e-9
        
        y = data.orientation.z*180/3.1416
        if y>self.yMax:
            self.yMax = y
        elif y<self.yMin:
            self.yMin =y
        vel = Twist()
        xvel1.input['yaw']=(y-self.setPoint)
        #xvel1.input['diff_yaw']= y-self.y_
        xvel1.compute()
        xvel = xvel1.output['velocity']
        
        #xvel = -pubx.getCorrection(setPoint,y)
        if xvel>self.xvelMax:
            self.xvelMax=xvel
        elif xvel<self.xvelMin:
            self.xvelMin = xvel
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x =0
        vel.angular.y = 0
        vel.angular.z = xvel
        self.pub.publish(vel)
        self.pub1.publish(y)
        #print "Max vel " + str(self.xvelMax) + " & Min vel " + str(self.xvelMin) + " Max y " + str(self.yMax*180/3.1416) +" & Min y" + str(self.yMin*180/3.1416)
        print "Velocity "+ str(xvel)+ " & yaw " + str(y)
        self.y_ = y
 
'''        if counter%10==0:
            plt.plot(y,time)
            plt.axis('equal')
            plt.draw()
            plt.pause(0.0000001)
        counter = counter+1'''

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