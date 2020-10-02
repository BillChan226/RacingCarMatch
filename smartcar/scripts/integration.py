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
from std_msgs.msg import Bool
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
global lidarLaunch
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
    
    intrinsicMat = np.array([[428.264533, -0.2570289, 295.1081],
                            [0, 427.8575, 253.5551],
                            [0, 0, 1]])

    distortionCoe = np.array([-0.38682604, 0.13534661, 8.18363975e-5, -2.8665336872e-4, 0])
    

    startx = 120
    starty = 50
    length_pers = 400
    width_pers = length_pers
    #srcps = np.float32([[(186,354), (85,431), (532,428), (406,347)]])
    #srcps_ramp = np.float32([[(27, 349), (177, 207), (452, 207), (599, 349)]])
    srcps_ramp = np.float32([[(189,237), (43,400), (539,386), (427,236)]])
    dstps = np.float32([[(startx, starty), (startx, starty + width_pers), (startx + length_pers, starty + width_pers), (startx + length_pers, starty)]])

    
    

    #Camera abandoned
    '''
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
    '''
    perspective_transform_matrix = cv2.getPerspectiveTransform(srcps_ramp, dstps)

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
        count = 0
        #摄像头端口号
        self.cap = cv2.VideoCapture(1)
        #self.cap.set(3, 1280)
        #self.cap.set(4, 720)
        
        #self.pub = rospy.Publisher('command', cmd, queue_size=1)
        self.pubI = rospy.Publisher('images', Image, queue_size=1)
        self.puborignialI = rospy.Publisher('orignial_images', Image, queue_size=1)
        rospy.init_node('laneDtctnAcmd', anonymous=True)
        rospy.Subscriber("laser_cmd", Twist, LScallback)
        rospy.Subscriber("has_red_light", Bool, RLcallback)
        self.rate = rospy.Rate(20)

        self.cvb = CvBridge()        

    def spin(self):
        global lidarLaunch, ramp_start, stop_judge_local
        threshold_value = 50
        global starter,pub,greenLight,aP,lastP, kernel
        global aP_kf
        global last_lane_base, last_LorR
	last_lane_base = -1
        last_LorR = 1
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
                    gray2 = cv2.Canny(binary, 50, 200)

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
                elif (not lidarLaunch):

                      
                    c1 = cv2.getTickCount()
                    kfObj = KalmanFilter()
        	    predictedCoords = np.zeros((2, 1), np.float32)
                    #img = cv2.pyrDown(img)
                    undstrt = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
                    self.puborignialI.publish(self.cvb.cv2_to_imgmsg(undstrt))                  
            	    #cv2.imshow('yuanshi',undstrt)
            	    #cv2.waitKey(1)
                    

        	    #gray0 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    gray = cv2.cvtColor(undstrt, cv2.COLOR_BGR2GRAY)
                    # lower = np.array([0,70,70])
                    # upper = np.array([100,255,255])
                    # yellow_mask = cv2.inRange(undstrt,lower,upper)
                    # yellow = cv2.bitwise_and(undstrt,undstrt,mask = yellow_mask)
                    # yellow_gray = cv2.cvtColor(yellow,cv2.COLOR_BGR2GRAY)
                    # ret,yellow_gray = cv2.threshold(yellow_mask,threshold_value,255,cv2.THRESH_BINARY)

                    # lab = cv2.cvtColor(undstrt,cv2.COLOR_BGR2Lab)
                    # lab_b = lab[:,:,2]
                    # if np.max(lab_b)>100:
                    #     lab_b =lab_b*(255/np.max(lab_b))
                    # yellow_lab = np.zeros_like(lab_b)
                    # yellow_lab[((lab_b>100)&(lab_b<=255))] = 1

                    # lower = np.array([0,70,70])
                    # upper = np.array([100,255,255])
                    # yellow_mask = cv2.inRange(undstrt,lower,upper)
                    # yellow = cv2.bitwise_and(undstrt,undstrt,mask = yellow_mask)
                    # yellow_gray = cv2.cvtColor(yellow,cv2.COLOR_BGR2GRAY)
                    # ret,yellow_gray = cv2.threshold(yellow_mask,threshold_value,255,cv2.THRESH_BINARY)
                    # HSV = cv2.cvtColor(undstrt, cv2.COLOR_BGR2HSV)
                    # min_th = np.all(HSV > np.array([15, 0, 160]), axis=2)
                    # max_th = np.all(HSV < np.array([150, 80, 255]), axis=2)
                    # yellow = np.logical_and(min_th, max_th)


                    # cv2.imshow('yellow_lab',yellow_lab)
                    # cv2.waitKey(1)                    
                    #cv2.imshow('yellow',yellow)
                    
                    #gray_warped = perspectiveTrans(gray)
                    #self.puborignialI.publish(self.cvb.cv2_to_imgmsg(gray_warped))
                    cv2.waitKey(1)

                    gray[:gray.shape[0]/3,:] = 0

                    gray_Blur = cv2.GaussianBlur(gray, (5,5),0)
                    #frame_gray = cv2.GaussianBlur(frame_gray, (5,5),0)
                    #cv2.imshow('gray',gray)
                    #cv2.waitKey(1)

                    #kernel = np.ones((10,10),np.uint8)              
                    #gray_Blur = cv2.morphologyEx(gray_Blur, cv2.MORPH_CLOSE, kernel, iterations = 1)


                    #origin_thr = np.zeros_like(gray_Blur)
                    #origin_thr[(gray_Blur <= 100)] = 255 

                    #_, origin_thr = cv2.threshold(gray_Blur,threshold_value,255, cv2.THRESH_BINARY_INV)
                    origin_thr = cv2.Canny(gray_Blur, 100, 200) 
                    
                    #print origin_thr.shape

                    origin_thr = cv2.dilate(origin_thr, np.ones((10,10), np.uint8))
                    origin_thr = cv2.erode(origin_thr, np.ones((12,12), np.uint8))
                    #self.pubI.publish(self.cvb.cv2_to_imgmsg(origin_thr)) 
                    #_, origin_thr = cv2.threshold(origin_thr,threshold_value,255, cv2.THRESH_BINARY)
                    #self.pubI.publish(self.cvb.cv2_to_imgmsg(origin_thr))
                    
                    #cv2.imshow('origin_thr',origin_thr)
                    #cv2.waitKey(1)

                    binary_warped =  perspectiveTrans(origin_thr)
                    binary_warped_ramp =  perspectiveTrans(gray)
                    #warped = perspectiveTrans(gray)
                    #self.pubI.publish(self.cvb.cv2_to_imgmsg(warped))
                    #continue
                    #每列元素相加，压缩成一行
                    histogram = np.sum(binary_warped[binary_warped.shape[0]*2/3:,:], axis=0) #//2
                    #midpoint = int(histogram.shape[0]/2)
                    
                    lane_base = np.argmax(histogram)


                    nwindows = 10
                    window_height = int(binary_warped.shape[0]/nwindows)
                    nonzero = binary_warped.nonzero()
                    nonzeroy = np.array(nonzero[0])
                    nonzerox = np.array(nonzero[1])
                    lane_current = lane_base
                    margin = 100
                    minpix = 1
                    lane_inds = []

                    binary_warped = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)


                    for window in range(nwindows):
                        win_y_low = binary_warped.shape[0] - (window + 1)*window_height
                        win_y_high = binary_warped.shape[0] - window*window_height 
                        win_x_low = lane_current - margin 
                        win_x_high = lane_current + margin 

                        cv2.rectangle(binary_warped, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0 ,0), 3)
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
                        ########
                        for i in range(binary_warped.shape[1]/5):
                            x = 5*i
                            y = int(a2*x**2 + a1*x + a0)
                            cv2.circle(binary_warped, (x, y), 3, (0, 0, 255), -1)


                    except:
                    	cam_cmd.angular.z = 0;
                    	#print(k*steerAngle*180/3.14)
                    	pub.publish(cam_cmd)
                    
                    
                    aveX = np.average(pixelX)
                    minY = np.min(pixelY)
                    #区分左右车道线,以计算截距
                    if (2*a2*aveX + a1) > 0 : #斜率大于0
                        if a2 > 0:
                            x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - binary_warped.shape[0] - (D - I) / y_cmPerPixel))**0.5))/(2*a2) #求截距
                        else :
                            x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - binary_warped.shape[0] - (D - I) / y_cmPerPixel))**0.5))/(2*a2)
                            
                    else : #斜率小于0
                        if a2 > 0:
                            x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - binary_warped.shape[0] -(D - I) / y_cmPerPixel))**0.5))/(2*a2) 
                        else :
                            x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - binary_warped.shape[0] -(D - I) / y_cmPerPixel))**0.5))/(2*a2)

                    if (x_intertcept > binary_warped.shape[1]/2.0):
                        LorR = -1; #RightLane
                        print('R')
                    else:
                        LorR = 1; #LeftLane
                        print('L')  

                                 


                    #frontDistance = np.argsort(pixelY)[int(len(pixelY)*8/10)]
                    #aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]


                    curvatureRadius_aim = 450.0/x_cmPerPixel
                    curvature_aim = 1/curvatureRadius_aim
                    
                    curvature_real = min(curvature_aim, 2*a2 - 0.000001)
                    print 'c: %.4f  2a2: %.4f'%(curvature_real, 2*a2)


                    if aveX < -a1/(2*a2):

                        curvature_x = (-((2*a2/curvature_real)**(2/3.0) - 1)**0.5 - a1)/(2*a2)
                    else:
                        curvature_x = (+((2*a2/curvature_real)**(2/3.0) - 1)**0.5 - a1)/(2*a2)

                    if curvature_x != curvature_x:
                        
                        print 'NaN'
                        pass
                    else:
                        cv2.line(binary_warped, (int(curvature_x), 0), (int(curvature_x) , binary_warped.shape[0]), (0, 0, 255), 3)

                    #curvature_y = 


                    aimLanePx = curvature_x
                    aimLanePy = a2*(curvature_x**2) + a1*curvature_x + a0
                    aimLaneP = [aimLanePx, aimLanePy]

                    if aimLanePy > binary_warped.shape[0]:
                        aimLanePy = binary_warped.shape[0]

                        delta = a1**2 - 4*a2*(a0 - aimLanePy)
                        if delta > 0:
                            delta = delta**0.5
                            if aveX < -a1/(2*a2):
                                print "LL"
                                aimLanePx = (-a1 - delta)/(2*a2)
                            else:
                                print "RR"
                                aimLanePx = (-a1 + delta)/(2*a2)
                            aimLaneP = [aimLanePx, aimLanePy]

                    if aimLanePy < minY:
                        aimLanePy = minY

                        delta = a1**2 - 4*a2*(a0 - aimLanePy)
                        if delta > 0:
                            delta = delta**0.5
                            if aveX < -a1/(2*a2):
                                print "LL"
                                aimLanePx = (-a1 - delta)/(2*a2)
                            else:
                                print "RR"
                                aimLanePx = (-a1 + delta)/(2*a2)
                            aimLaneP = [aimLanePx, aimLanePy]



                           
                            
   
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

                    #binary_warped = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)

                    if aP[0] != aP[0] or aP[1] != aP[1]:
                        continue
                   
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
                    if steerAngle < 0:
                        cam_cmd.angular.z = k1*steerAngle
                    else:
                        cam_cmd.angular.z = k2*steerAngle
                    print 'steerAngle: %.5f'%(steerAngle)
                    #print(k*steerAngle*180/3.14)

                    #rospy.spinOnce()
                    if ramp_start == True and stop_judge_local == False:
                        cam_cmd.angular.z = 0.0
                        cam_cmd.linear.x = 0.1
                    pub.publish(cam_cmd)
                    self.pubI.publish(self.cvb.cv2_to_imgmsg(binary_warped_ramp))
                    c2 = cv2.getTickCount()
                    print 'time'
                    print (c2 - c1)/cv2.getTickFrequency()
                    
                else:
                    pub.publish(cam_cmd)
                        
            self.rate.sleep()

        self.cap.release()


def RLcallback(stop_judge):
    global stop_judge_local, ramp_start
    stop_judge_local = stop_judge.data
    if stop_judge.data == True:
        cam_cmd.linear.x = 0
        ramp_start = True
    else:
        if ramp_start == True:
            cam_cmd.linear.x = 0.1
        else:
            cam_cmd.linear.x = 0.5

def LScallback(laser_cmd):
    global stop_judge_local, lidarLaunch
    if stop_judge_local == True:
        return
    
    if laser_cmd.linear.z > 0:  #laser control steer 
        lidarLaunch = True 
        cam_cmd.angular.z = laser_cmd.angular.z
    else:
        lidarLaunch = False
        pass
    
    cam_cmd.linear.x = laser_cmd.linear.x
        

if __name__ == '__main__':
    #device = rospy.get_param('device', 1)
    #width = rospy.get_param('width', 1280)
    #height = rospy.get_param('height', 720)
    #rates = rospy.get_param('rates', 10)

    initial_parameters()
    #距离映射
    x_cmPerPixel = 80/400.0
    y_cmPerPixel = 80/400.0
    roadWidth = 65.0 / x_cmPerPixel #80.0
    y_offset = 0.0 #cm
    #


    aP = [0.0, 0.0]
    lastP = [0.0, 0.0]
    aP_kf=[0.0,0.0]
    Timer = 0

    #轴间距
    I = 61.5
    #图像坐标系底部与车后轮轴心间距
    D = 126
    #计算cmdSteer的系数，舵机转向与之前相反，此处用正数
    k1 = 3.6		 # 3.6
    k2 = 3.6
    #steerAngle, cmdSteer;
    cam_cmd = Twist()
    cam_cmd.linear.x = 0.5
    starter = False
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    greenLight = 0
    global stop_judge_local, ramp_start
    stop_judge_local = False
    ramp_start = False
    
   
    try:
        cam = camera()
        cam.spin()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
