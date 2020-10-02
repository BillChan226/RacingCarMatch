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
    if max_val < 0.55:
        return target, False
    if methods == cv.TM_SQDIFF_NORMED:
        tl = min_loc
    else:
        tl = max_loc
    br = (tl[0]+tw, tl[1]+th)   #br是矩形右下角的点的坐标
    #cv.rectangle(target, tl, br, (0, 0, 255), 2)
    return target, True
    
class rampDetector:
    def __init__(self):
        self.has_ramp = Bool()
        self.cvb = CvBridge()
        currentpath, _ = os.path.split(os.path.abspath(sys.argv[0]))

        self.ramppath = os.path.join(currentpath, 'ramp_sample.png')

        self.ramppub = rospy.Publisher('has_ramp', Bool, queue_size=1)

        rospy.Subscriber('orignial_images', Image, self.callback)

        rospy.init_node('ramp_detection', anonymous=True)
        
    def callback(self, imgmsg):
        img = self.cvb.imgmsg_to_cv2(imgmsg)

        ramptpl =cv.imread(self.ramppath)

        ramp_result, has_ramp = template_demo(img, ramptpl)

        self.has_ramp.data = has_ramp
  
        print('has_ramp',self.has_ramp.data)

        self.ramppub.publish(self.has_ramp)

        #cv.imshow("ramp", ramp_result)

        #cv.waitKey(1)

if __name__ == "__main__":
    try:
        detector = rampDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()
