#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
#import sys, time
import rospy
#import roslib
import cv2
#import numpy as np
#from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "image_resizer_node"
#SUB_TOPIC = "/image"
SUB_TOPIC = "/image_raw"
PUB_TOPIC = "image_preproc_resized"
QUEUE_SIZE = 1
DEFAULT_HEIGHT = 216
DEFAULT_WIDTH = 384


class ImageReziserNode:
    def initial_parameters(self):
        global intrinsicMat
        global distortionCoe
        #global perspective_transform_matrix 
        #global kernel

        intrinsicMat = np.array([[669.0672, -0.2097, 490.6801],
                            [0, 671.0723, 283.2345],
                            [0, 0, 1]])

        distortionCoe = np.array([-0.3739,0.1119,3.5478e-04,0.002, 0])
       

        startx = 280
        starty = 220
        length_pers = 400
        width_pers = length_pers
        srcps = np.float32([[(289,250), (93,415), (870,419), (680,256)]])
        #srcps_ramp = np.float32([[(27, 349), (177, 207), (452, 207), (599, 349)]])
        dstps = np.float32([[(startx, starty), (startx, starty + width_pers), (startx + length_pers, starty + width_pers), (startx + length_pers, starty)]])

        #perspective_transform_matrix = cv2.getPerspectiveTransform(srcps, dstps)

        #kernel = np.ones((3,3),np.uint8)

    def __init__(self, node_name, sub_topic, pub_topic):
        self.bridge = CvBridge()
        self.initial_parameters()
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)

        rospy.init_node(node_name, anonymous=True)
        #self.image_sub = rospy.Subscriber(sub_topic, CompressedImage, self.callback)
	self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
        self.rate = rospy.Rate(20)
        rospy.spin()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.pyrDown(cv_image)
            undstrt = cv2.undistort(cv_image, intrinsicMat, distortionCoe, None, intrinsicMat)

        except CvBridgeError as e:
            rospy.logerr(e)

        #height = 1080
        #width = 1920
        #cv_image = cv2.resize(cv_image, (width, height), 0, 0, 0)
        
        #print(cv_image.shape)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(undstrt, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)


def main():
    try:
        ImageReziserNode(NODE_NAME, SUB_TOPIC, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
