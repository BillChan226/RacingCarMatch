#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
import os
import sys
import glob
import numpy as np
import math


from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

intrinsicMat = []
distortionCoe = []
prspct_trans_mat = []
perspective_transform_matrix = []
kernel = []

#not detect the green light
starter = True
#ignore the lidar
lidarLaunch = False

class KalmanFilter:

    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        return predicted

def initial_parameters():
    global intrinsicMat
    global distortionCoe
    global perspective_transform_matrix 
    global kernel

    intrinsicMat = np.array([[426.140302, -6.33336 , 330.827470],
                            [0, 426.26858, 241.74322],
                            [0, 0, 1]])

    distortionCoe = np.array([-0.38682604, 0.13534661, 8.18363975e-5, -2.8665336872e-4, 0])

    startx = 120
    starty = 40
    length_pers = 400
    width_pers = length_pers
    srcps = np.float32([[(227, 347), (116, 415), (564, 410), (457, 345)]])
    srcps_ramp = np.float32([[(27, 349), (177, 207), (452, 207), (599, 349)]])
    dstps = np.float32([[(startx, starty), (startx, starty + width_pers), (startx + length_pers, starty + width_pers), (startx + length_pers, starty)]])
    perspective_transform_matrix = cv2.getPerspectiveTransform(srcps, dstps)

    kernel = np.ones((3,3),np.uint8)



def perspectiveTrans(img):

    global perspective_transform_matrix    

    if perspective_transform_matrix==[]:
        print"Transform failed!"
        return img
    else:
        bird_view_img = cv2.warpPerspective(img, perspective_transform_matrix, img.shape[1::-1], flags=cv2.INTER_LINEAR)
        return bird_view_img



class camera:
    def __init__(self):

        #摄像头端口号
        self.cap = cv2.VideoCapture(1)
        #self.cap.set(3, 1280)
        #self.cap.set(4, 720)
        
        #self.pub = rospy.Publisher('command', cmd, queue_size=1)
        self.pubI = rospy.Publisher('images', Image, queue_size=1)
        self.puborignialI = rospy.Publisher('orignial_images', Image, queue_size=1)
        rospy.init_node('laneDtctnAcmd', anonymous=True)
        #rospy.Subscriber("LScmd", cmd, LScallback)
        self.rate = rospy.Rate(10)

        self.cvb = CvBridge()        

    def spin(self):
        threshold_value = 185
        global starter,pub,greenLight,aP,lastP, kernel
        global aP_kf
        while not rospy.is_shutdown():
            ret, img = self.cap.read()
            if ret == True:
                if(False):#not starter):
                    #detect the green light

                    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
                    lower=np.array([80,50,180])
                    upper=np.array([99,255,255])

                    mask=cv2.inRange(hsv,lower,upper)
                    target = cv2.bitwise_and(img, img, mask=mask)
                    gray = cv2.cvtColor(target,cv2.COLOR_BGR2GRAY)
                    ret,binary = cv2.threshold(mask,127,255,cv2.THRESH_BINARY)
                    gray2 = cv2.Canny(binary, 100, 200)

                    #cv2.imshow('canny',gray2)
                    #cv2.waitKey(1000)
                    #try:
                    circles1=cv2.HoughCircles(gray2,cv2.HOUGH_GRADIENT,1,600,param1=255,param2=30,minRadius=20,maxRadius=140)
                    #print(type(circles1))
                    #except Exception as result:
                    if(circles1 is None):
                        greenLight=0
                       
                    else:
                        greenLight=1

                    if (greenLight == 1):
                        print("Start!!!!!!!!!!!!,  %d", greenLight)  
                        starter = True
                    #starter = True
                elif(not lidarLaunch):
                    kfObj = KalmanFilter()
        	    predictedCoords = np.zeros((2, 1), np.float32)
                    undstrt = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
            	    cv2.imshow('yuanshi',undstrt)
            	    cv2.waitKey(1)
                    self.puborignialI.publish(self.cvb.cv2_to_imgmsg(img))

        
                    gray = cv2.cvtColor(undstrt, cv2.COLOR_BGR2GRAY)
                    gray_warped = perspectiveTrans(gray)

                    gray_Blur = gray
                    #cv2.imshow('gray',gray)
                    #cv2.waitKey(1)

                    #kernel = np.ones((10,10),np.uint8)              
                    #gray_Blur = cv2.morphologyEx(gray_Blur, cv2.MORPH_CLOSE, kernel, iterations = 1)


                    #origin_thr = np.zeros_like(gray_Blur)
                    #origin_thr[(gray_Blur <= 100)] = 255 

                    _, origin_thr = cv2.threshold(gray_Blur,threshold_value,255, cv2.THRESH_BINARY)
                    #origin_thr = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,155,35)
                    #cv2.imshow('origin_thr',origin_thr)
                    #cv2.waitKey(1)

                    binary_warped =  perspectiveTrans(origin_thr)
                    #每列元素相加，压缩成一行
                    histogram = np.sum(binary_warped[binary_warped.shape[0]*4/5:,:], axis=0) #//2
                    #midpoint = int(histogram.shape[0]/2)
                    
                    lane_base = np.argmax(histogram)

                    nwindows = 5;
                    window_height = int(binary_warped.shape[0]/nwindows)
                    nonzero = binary_warped.nonzero()
                    nonzeroy = np.array(nonzero[0])
                    nonzerox = np.array(nonzero[1])
                    lane_current = lane_base
                    margin = 100
                    minpix = 1
                    lane_inds = []


                    for window in range(nwindows):
                        win_y_low = binary_warped.shape[0] - (window + 1)*window_height
                        win_y_high = binary_warped.shape[0] - window*window_height 
                        win_x_low = lane_current - margin 
                        win_x_high = lane_current + margin 
                        good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                        
                        lane_inds.append(good_inds)
                        if len(good_inds) > minpix:
                            lane_current = int(np.mean(nonzerox[good_inds])) ####

                    lane_inds = np.concatenate(lane_inds)#数组拼接

                    pixelX = nonzerox[lane_inds]
                    pixelY = nonzeroy[lane_inds]

                    # calculate the aimPoint
                    
                    if (pixelX.size == 0):
                        continue
                    try:
			a2,a1,a0 = np.polyfit(pixelX, pixelY, 2);
                    except:
                    	cam_cmd.angular.z = 0;
                    	print(k*steerAngle*180/3.14)
                    	pub.publish(cam_cmd)
                    
                    
                    aveX = np.average(pixelX)
                    #区分左右车道线,以计算截距
                    if (2*a2*aveX + a1) > 0 : #斜率大于0
                        if a2 > 0:
                            x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - (D - I) / y_cmPerPixel))**0.5))/(2*a2) #求截距
                        else :
                            x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - (D - I) / y_cmPerPixel))**0.5))/(2*a2)
                            
                    else : #斜率小于0
                        if a2 > 0:
                            x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - (D - I) / y_cmPerPixel))**0.5))/(2*a2) 
                        else :
                            x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - (D - I) / y_cmPerPixel))**0.5))/(2*a2)

                    if (x_intertcept > binary_warped.shape[1]/2.0):
                        LorR = -1; #RightLane
                        print('R')
                    else:
                        LorR = 1; #LeftLane
                        print('L')                


                    frontDistance = np.argsort(pixelY)[int(len(pixelY)/5)]
                    aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]
                    
                    #计算aimLaneP处斜率，从而得到目标点的像素坐标
                    lanePk = 2*a2*aimLaneP[0] + a1
                    if (lanePk > 500 or lanePk < -500):
                        aP[0] = aimLaneP[0] + LorR*roadWidth/2
                        aP[1] = aimLaneP[1]
                    else :
                        k_ver = -1/lanePk
                        theta = math.atan(k_ver)
                        aP[0] = aimLaneP[0] + math.cos(theta)*(LorR)*roadWidth/2
                        aP[1] = aimLaneP[1] + math.sin(theta)*(LorR)*roadWidth/2

                    predictedCoords = kfObj.Estimate(aP[0], aP[1])
                    aP_kf[0] = predictedCoords[0][0]
                    aP_kf[1] = predictedCoords[1][0]

                    Y_half = np.argsort(pixelY)[int(len(pixelY)/2)]
                    X_half = np.argsort(pixelX)[int(len(pixelY)/2)]
                    
                    aP_kf[0] = pixelX[X_half]+LorR*200
                    aP_kf[1] = pixelY[Y_half]+LorR*50

                    #aP[0] = aP_kf[0]
                    #aP[1] = aP_kf[1]

                    binary_warped = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)
                    cv2.circle(binary_warped, (int(aP[0]), int(aP[1])), 20, (0, 0, 255), -1)
                    cv2.circle(binary_warped, (int(aP_kf[0]), int(aP_kf[1])), 20, (0, 255, 0), -1)

                    

                    #计算目标点的真实坐标
                    aP[0] = (aP[0] - binary_warped.shape[1]/2.0)*x_cmPerPixel
                    aP[1] = (binary_warped.shape[0] - aP[1])*y_cmPerPixel + y_offset
                    
                    
                    if(lastP[0] > 0.001 and lastP[1] > 0.001):
                        if(((aP[0]-lastP[0])**2 + (aP[1]-lastP[1])**2 > 1500) and Timer < 4 ): #To avoid the mislead by walkers
                            aP = lastP
                            Timer = Timer + 1
                        else:
                            Timer = 0

                    lastP = aP 
                    steerAngle = math.atan(2*I*aP[0]/(aP[0]*aP[0]+(aP[1]+D)*(aP[1]+D)));

                    cam_cmd.angular.z = k*steerAngle;
                    print(k*steerAngle*180/3.14)

                    #rospy.spinOnce()
                    pub.publish(cam_cmd)
                    self.pubI.publish(self.cvb.cv2_to_imgmsg(binary_warped))
                    
                    
                #else:
                    
            self.rate.sleep()

        self.cap.release()


def LScallback(cmd):
    global lidarLaunch, pub
    if((not lidarLaunch)  and (cmd.angular.z  > 0.0001 or cmd.angular.z  < -0.0001 )):    
        lidarLaunch = True

    if(lidarLaunch):
        pub.publish(cmd)
        if(cmd.linear.x < -0.001):
            lidarLaunch = False
    else:
        cam_cmd.linear.x = cmd.linear.x
        

if __name__ == '__main__':
    #device = rospy.get_param('device', 1)
    #width = rospy.get_param('width', 1280)
    #height = rospy.get_param('height', 720)
    #rates = rospy.get_param('rates', 10)

    initial_parameters()
    #距离映射
    x_cmPerPixel = 80/400.0
    y_cmPerPixel = 80/400.0
    roadWidth = 80.0 / x_cmPerPixel #80.0
    y_offset = 0.0 #cm
    #


    aP = [0.0, 0.0]
    lastP = [0.0, 0.0]
    aP_kf=[0.0,0.0]
    Timer = 0

    #轴间距
    I = 61.5
    #图像坐标系底部与车后轮轴心间距
    D = 151
    #计算cmdSteer的系数，舵机转向与之前相反，此处用正数
    k = 2.8 

    #steerAngle, cmdSteer;
    cam_cmd = Twist()
    cam_cmd.linear.x = 0.3
    starter = False
    lidarLaunch = False
    pub = rospy.Publisher('lane_vel', Twist, queue_size=1)

    greenLight = 0
    
    try:
        cam = camera()
        cam.spin()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
