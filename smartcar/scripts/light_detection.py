#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 18 20:51:41 2018

@author: jjldr
"""
import os
import sys
import glob
import cv2 as cv

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def template_demo(target,tpl):
    
    #methods = [cv.TM_SQDIFF_NORMED, cv.TM_CCORR_NORMED, cv.TM_CCOEFF_NORMED]   #3种模板匹配方法
    methods=cv.TM_CCOEFF_NORMED
    th, tw = tpl.shape[:2]
    
    result = cv.matchTemplate(target, tpl, methods)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
    print(max_val)
    if max_val < 0.7:
        return target, False
    if methods == cv.TM_SQDIFF_NORMED:
        tl = min_loc

    else:
        tl = max_loc
    br = (tl[0]+tw, tl[1]+th)   #br是矩形右下角的点的坐标
    cv.rectangle(target, tl, br, (0, 0, 255), 2)
    return target, True
    
class lightDetector:
    def __init__(self):
        self.has_red_light = Bool()
        self.has_green_light = Bool()
        self.cvb = CvBridge()
        rospy.init_node('light_detection', anonymous=True)
        currentpath, _ = os.path.split(os.path.abspath(sys.argv[0]))

        self.redpath = os.path.join(currentpath, 'red_sample.jpg')
        self.greenpath = os.path.join(currentpath, 'green_sample.jpg')

        self.redpub = rospy.Publisher('has_red_light', Bool, queue_size=1)
        self.greenpub = rospy.Publisher('has_green_light', Bool, queue_size=1)

        rospy.Subscriber('orignial_images', Image, self.callback)

        
        
    def callback(self, imgmsg):
        img = self.cvb.imgmsg_to_cv2(imgmsg)

        redlighttpl =cv.imread(self.redpath)
        greenlighttpl =cv.imread(self.greenpath)

        red_result, has_red_light = template_demo(img, redlighttpl)

        green_result, has_green_light = template_demo(img, greenlighttpl)

        self.has_red_light.data = has_red_light
        self.has_green_light.data = has_green_light

  
        print('has_red',self.has_red_light.data)
        print('has_green',self.has_green_light.data)

        self.redpub.publish(self.has_red_light)
        self.greenpub.publish(self.has_green_light)

        #cv.imshow("red", ramp_result)

        #cv.waitKey(1)

if __name__ == "__main__":
    try:
        detector = lightDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()
