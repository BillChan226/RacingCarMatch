#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleDetector:
    def __init__(self):
        self.min_dist_left = 0
        self.min_dist_right = 0
        self.k = 1.0
        self.flag = 0
        self.has_obs = Bool()

        rospy.init_node('obstacle_detection', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('has_obs', Bool, queue_size=1)
        print 'LiDAR is OK'
    def callback(self, msg):
        minIndex_left = 0
        minIndex_right = 1360
	
        # 雷达正前为1440
        
        for i in range (0, 80):
            if msg.ranges[i] < msg.ranges[minIndex_left] and msg.ranges[i] > 0.05:
                minIndex_left = i
        for j in range (1360,1440):
            if msg.ranges[i] < msg.ranges[minIndex_right] and msg.ranges[i] > 0.05:
                minIndex_right = j
        
        self.min_dist_left = msg.ranges[minIndex_left]
        self.min_dist_right = msg.ranges[minIndex_right]

	self.has_obs.data=False
        self.pub.publish(self.has_obs)
        print ('ok')
        #检测正前方40度有无小于0.6米的障碍物
        if self.min_dist_left < 0.25 or self.min_dist_right < 0.25 :
            print('stop')
            self.has_obs.data=True
            self.pub.publish(self.has_obs)
        else:
            self.has_obs.data=False
            self.pub.publish(self.has_obs)
            print ('ok')

if __name__ == '__main__':
    try:
        detector = ObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
