#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import os
import sys  
import tty, termios  
#import roslib; roslib.load_manifest('smartcar_teleop')  
import rospy
from geometry_msgs.msg import Twist  
from std_msgs.msg import String  

 

# 全局变量  

cmd = Twist()  

pub = rospy.Publisher('cmd_vel', Twist)  
#speedstep = 0.1

#steerstep = 0.05



def keyboardLoop():  

    #初始化  

    rospy.init_node('smartcar_teleop')  

    rate = rospy.Rate(rospy.get_param('~hz', 10))
    speed = 0
    turn = 0
 

    #速度变量  

   # walk_vel_ = rospy.get_param('walk_vel', 0.05)  

    #run_vel_ = rospy.get_param('run_vel', 0.1)  

    #yaw_rate_ = rospy.get_param('yaw_rate', 0.05)  

    #yaw_rate_run_ = rospy.get_param('yaw_rate_run', 0.1)  

 

    #max_tv = walk_vel_  

    #max_rv = yaw_rate_  

 

    #显示提示信息  

    print "Reading from keyboard"  

    print "Use WASD keys to control the robot"  

    print "Press Caps to move faster"  

    print "Press q to quit"  

 

    #读取按键循环  

    while not rospy.is_shutdown():  

        fd = sys.stdin.fileno()  

        old_settings = termios.tcgetattr(fd)  

        #不产生回显效果  

        #old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO  

        try :  

            tty.setraw( fd )  

            ch = sys.stdin.read( 1 )  

        finally :  

            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  

 

        if ch == 'w':  

            speed = speed + 1  

           # turn = 0  

        elif ch == 's':  

  

            speed = speed - 1  

            #turn = 0  

        elif ch == 'a':  

 

            #speed = 0  

            turn = turn + 1  

        elif ch == 'd':  

    

            #speed = 0  

            turn = turn -1  

        elif ch == 'W':  

        

            speed = speed + 1  

           # turn = 0  

        elif ch == 'S':  

       

            speed = speed - 1  

            #turn = 0  

        elif ch == 'A':  

       

            #speed = 0  

            turn = turn + 1  

        elif ch == 'D':  

            

            #speed = 0  

            turn = turn - 1  

        elif (ch == 'z') or (ch == 'Z'):
            turn = 7
        elif (ch == 'c') or (ch == 'C'):
            turn = -7   
        elif ch == 'q':  

            exit() 

        else:  
  

            speed = 0  

            turn = 0  

 

        #发送消息  
        cmd.linear.x = speed * 0.1;  

        cmd.angular.z = -turn * 0.2;  

        pub.publish(cmd)  

        rate.sleep()  

        #停止机器人  

        #stop_robot();  

 

def stop_robot():  

    cmd.linear.x = 0.0  

    cmd.angular.z = 0.0  

    pub.publish(cmd)  

 

if __name__ == '__main__':  

    try:  

        keyboardLoop()  

    except rospy.ROSInterruptException:  

        pass  
