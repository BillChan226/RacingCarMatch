#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import time
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class rplidarDetector:
    def __init__(self):
        self.lidar_start = Bool()
        self.lidar_vel = Twist()
        rospy.init_node('rplidar_detection', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.velpub = rospy.Publisher('lidar_vel', Twist, queue_size=1)
        self.startpub = rospy.Publisher('lidar_start', Bool, queue_size=1)
        print 'LiDAR is OK'

    def road_detection(self, msg):
        #雷达正前方为1440
        ############################ 找左侧距离最近的点
        right_min = 25
        right_min_index = 0
        for i in range(0, 240):
            if msg.ranges[i] < right_min and msg.ranges[i] > 0.05:
                right_min = msg.ranges[i]
                right_min_index = i

        ############################ 找右侧距离最近的点
        left_min = 25
        left_min_index = 1200
        for i in range(1200, 1440):
            if msg.ranges[i] < left_min and msg.ranges[i] > 0.05:
                left_min = msg.ranges[i]
                left_min_index = i

        ############################ 判断左右哪边离障碍物更近
        offset = left_min - right_min
        if (left_min < 0.6 or right_min < 0.6):
            return True,offset
        else:
            return False,offset

    def callback(self, msg):
        res,offset = self.road_detection(msg)
        print('offset',offset)
        print('result',res)
        self.lidar_start.data = res
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = offset*2.5
        self.velpub.publish(twist)
        self.startpub.publish(self.lidar_start)

if __name__ == '__main__':
    try:
        detector = rplidarDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
