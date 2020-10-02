#!/usr/bin/env python
# -*- coding: UTF-8 -*-


import cv2
import time
import numpy as np

cap = cv2.VideoCapture(0)#打开相机
methods = [cv2.TM_SQDIFF_NORMED, cv2.TM_CCORR_NORMED, cv2.TM_CCOEFF_NORMED]   #3种模板匹配方法
md = methods[0]
tpl = cv2.imread('greenlight.png')

i = 0
while(True):
    start=time.clock() 
    ret,frame = cap.read()#捕获一帧图像
    if ((i % 10) == 0):
        th, tw = tpl.shape[:2]
        result = cv2.matchTemplate(frame, tpl, md)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        tl = min_loc
        print min_val

        br = (tl[0]+tw, tl[1]+th)   #br是矩形右下角的点的坐标
        cv2.rectangle(frame, tl, br, (0, 0, 255), 2)
        cv2.imshow("match-" + np.str(md), frame)
        cv2.waitKey(25)
        '''
        if ret:
            cv2.imshow('frame',frame)
            cv2.waitKey(25)
        else:
            break
        '''
        if (min_val < 0.18):
            break
            
        else:
            print 'False'
        end=time.clock()
        print("time:%s"%(end-start))
    i = i + 1
print 'True'





cap.release()#关闭相机
cv2.destroyAllWindows()#关闭窗口
