
import cv2 as cv
import numpy as np
 
image = cv.imread("D:/light samples/light18.png") #在此处放图片文件位置

gray_img =cv.cvtColor(image,cv.COLOR_BGRA2GRAY)

img  = cv.medianBlur(gray_img, 7)  #进行中值模糊，去噪点
cimg = cv.cvtColor(img, cv.COLOR_GRAY2BGR)

circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT, 1, 60, param1=190, param2=59, minRadius=0, maxRadius=0)

circles = np.uint16(np.around(circles))
print(circles)

for i in circles[0,:]: #遍历矩阵每一行的数据，在原始图中画圆
    cv.circle(image, (i[0],i[1]),i[2],(0,255,0) ,2)
    cv.circle(image, (i[0], i[1]),2, (0,0,255) ,3)

cv.imshow("gray_img",image)
cv.waitKey(0)
cv.destroyAllWindows()
