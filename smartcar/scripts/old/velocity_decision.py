#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class velocityDecide:
    def __init__(self):
        self.cmd_vel = Twist()
        self.lane_vel = Twist()
        self.has_obs = Bool()
        self.lidar_start = Bool()
        self.lidar_vel = Twist()
        self.has_green_light= Bool()
        self.has_red_light= Bool()
       
        rospy.init_node('velocity_decision', anonymous=True)
        #视觉循线
        rospy.Subscriber('lane_vel', Twist, self.lanecallback)
        #障碍物检测
        rospy.Subscriber('has_obs', Bool, self.obscallback)
        #隧道检测
        rospy.Subscriber('lidar_start', Bool, self.rpstartcallback)
        #隧道循线
        rospy.Subscriber('lidar_vel', Twist, self.rplidarcallback)
        #红绿灯检测
        rospy.Subscriber('has_green_light', Bool, self.lightcallback)
        rospy.Subscriber('has_red_light', Bool, self.lightcallback)

        #最终的速度发布
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
    
    def lanecallback(self, msg):
        self.lane_vel = msg
    def obscallback(self, msg):
        self.has_obs = msg
    def rplidarcallback(self,msg):
        self.lidar_vel = msg
    def rpstartcallback(self,msg):
        self.lidar_start = msg
    def lightcallback(self,msg):
        self.has_red_light = msg
        self.has_green_light = msg
    def spin(self):
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.lidar_start.data == False:
            	self.cmd_vel = self.lane_vel
            	print('lane_vel',self.lane_vel.linear.x)
            '''
            if self.has_obs.data == True:
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
            '''
            if self.lidar_start.data == True:
                self.cmd_vel = self.lidar_vel
                print('lidar_vel',self.lidar_vel.linear.x)
            '''
            if self.has_green_light.data == True:
            if self.has_red_light.data == True:
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
            '''
            print('final vel',self.cmd_vel.linear.x)
            self.pub.publish(self.cmd_vel)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        decide = velocityDecide()
        decide.spin()
    except rospy.ROSInterruptException:
        pass

    
        
