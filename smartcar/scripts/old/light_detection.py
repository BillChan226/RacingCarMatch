#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
import os
import sys
import glob
import numpy as np
import math
from std_msgs.msg import Bool


from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist



class trafficLightDetector:
    def __init__(self):
    	global greenLight
    	self.has_red_light = Bool()
        self.has_green_light = Bool()
        self.cvb = CvBridge()
        self.redlightpub = rospy.Publisher('has_red_light', Bool, queue_size=1)
        self.greenlightpub = rospy.Publisher('has_green_light', Bool, queue_size=1)
        #rospy.Subscriber('images', Image, self.callback)
        
        rospy.init_node('traffic_light_detection', anonymous=True)
        #摄像头端口号
        #self.cap = cv2.VideoCapture(1)
        #ret, img = self.cap.read()
        img = cv2.imread('2.jpg') 
        print("start traffic light detection")
        predictedCoords = np.zeros((2,  1), np.float32)
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        #绿色交通灯mask值
        green_lower=np.array([80,50,180])
        green_upper=np.array([99,255,255])
        #红色交通灯mask值
        red_lower=np.array([0,43,46])
        red_upper=np.array([10,255,255])
        #红绿灯提取，霍夫圆变换
        green_mask=cv2.inRange(hsv,green_lower,green_upper)
        red_mask=cv2.inRange(hsv,red_lower,red_upper)

        green_target = cv2.bitwise_and(img, img, mask=green_mask)
        red_target = cv2.bitwise_and(img, img, mask=red_mask)

        green_gray = cv2.cvtColor(green_target,cv2.COLOR_BGR2GRAY)
        red_gray = cv2.cvtColor(red_target,cv2.COLOR_BGR2GRAY)

        g_ret,g_binary = cv2.threshold(green_mask,127,255,cv2.THRESH_BINARY)
        r_ret,r_binary = cv2.threshold(red_mask,127,255,cv2.THRESH_BINARY)

        g_gray2 = cv2.Canny(g_binary, 100, 200)
        r_gray2 = cv2.Canny(r_binary, 100, 200)
        
       	circles_red=cv2.HoughCircles(g_gray2,cv2.HOUGH_GRADIENT,1,600,param1=255,param2=30,minRadius=50,maxRadius=200)
        circles_green=cv2.HoughCircles(r_gray2,cv2.HOUGH_GRADIENT,1,600,param1=255,param2=30,minRadius=50,maxRadius=200)
       	#cv2.imshow('green',g_gray2)
        #cv2.waitKey(100)
        print(circles_green)
        print(circles_red)
        if(circles_green is None):
        	greenLight = 0
        else:
        	greenLight = 1
        	print("greenlight detected!!")
        if(circles_red is None):
        	redLight = 0
        else:
        	redLight = 1
        	print("redlight detected!!")
    def callback(self):
        self.redlightpub.publish(self.has_red_light)
        self.greenlightpub.publish(self.has_green_light)
        cv.imshow("tracffic_light", light_result)
        cv.waitKey(1)

if __name__ == "__main__":
	greenLight = 0
	try:
		detector = trafficLightDetector()
		rospy.spin()
	except rospy.ROSInterruptException:
		cv.destroyAllWindows()