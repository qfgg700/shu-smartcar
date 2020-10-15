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
def initial_parameters():
    global intrinsicMat
    global distortionCoe
    '''
    intrinsicMat = np.array([[1346, 0.6211 , 968.2755],
                            [0, 1344.7, 507.2217],
                            [0, 0, 1]])

    distortionCoe = np.array([-0.37,0.1097,0.0068,0.0034, 0])
    '''
class trafficLightDetector:
    '''
    def initial_parameters(self):
        global intrinsicMat
        global distortionCoe
           #1w摄像头的参数
        intrinsicMat = np.array([[1346, 0.6211 , 968.2755],
                            [0, 1344.7, 507.2217],
                            [0, 0, 1]])

        distortionCoe = np.array([-0.37,0.1097,0.0068,0.0034, 0])
           #普通摄像头的参数
        intrinsicMat = np.array([[428.264533,-0.2570289,295.1081],
                            [0, 427.8575,253.5551],
                            [0, 0, 1]])

        distortionCoe = np.array([-0.38682604,0.13534661,8.18363975e-5,-2.866536872e-4 0])
    '''
    def __init__(self):
        #self.initial_parameters()
        has_red_light = Bool()
        self.rawImg = rospy.Publisher('raw',Image,queue_size=1)
        self.cvb = CvBridge()
        self.redlightpub = rospy.Publisher('has_red_light', Bool, queue_size=1)
        self.Redlight = rospy.Publisher('redLight',Image,queue_size = 1)
        rospy.init_node('traffic_light_detection', anonymous=True)
        print("detect inital success")
        rate = rospy.Rate(20) # 10hz
        rospy.Subscriber('orignial_images', Image, self.callback)
    
    def callback(self,imgmsg):
        c1 = cv2.getTickCount()
        img = self.cvb.imgmsg_to_cv2(imgmsg)
        #undstrt = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
        #judge1 = self.line_detection(img)    #判断是否存在停车线
        judge1 =True
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)   #转化为HSV格式图像
        red_lower = np.array([0,85,220])
        red_upper = np.array([20,255,255])
        red_mask = cv2.inRange(hsv,red_lower,red_upper)   #遮蔽
        red_target = cv2.bitwise_and(img,img,mask = red_mask)
            #for i in range(red_mask.shape[1]):
            #    for j in range(red_mask.shape[0]):
            #        if red_target[j][i][0] + red_target[j][i][1]+red_target[j][i][2] != 0 :
            #            red_target[j][i] = [10,63,255]
        element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        red_target = cv2.erode(red_target,element)
        red_target = cv2.dilate(red_target,element)
        red_gray = cv2.cvtColor(red_target,cv2.COLOR_BGR2GRAY)
        r_ret,r_binary = cv2.threshold(red_mask,127,255,cv2.THRESH_BINARY)
        r_gray2 = cv2.Canny(r_binary, 100, 200) 
       	#circles_red=cv2.HoughCircles(r_gray2,cv2.HOUGH_GRADIENT,1,600,param1=255,param2=30,minRadius=15,maxRadius=100)
        #if(circles_red is None):
        #    redLight = 0sedx
        #    has_red_light = False
        #else:
        #    redLight = 1
        #    has_red_light = True

        #self.Redlight.publish(CvBridge().cv2_to_imgmsg(r_gray2)) 
        g = r_gray2[:,:] == 255
        count = len(r_gray2[g])
        if count>700:
            has_red_light = True
            print("red_light detected!")
        else:
            has_red_light = False
        print(count)
        stop_judge = has_red_light and judge1  #存在红灯且存在停车线，则停车
        self.redlightpub.publish(stop_judge)
        print(stop_judge)
        c2 = cv2.getTickCount()
        cycle_period = (c2 - c1) / cv2.getTickFrequency()
        print'cycle: %.4f'%(cycle_period)
      


    def line_detection(self,img):
        result = img.copy()
        result = result[380:480,:]
        line_count = 0
        result = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
        r_ret,r_binary = cv2.threshold(result,127,255,cv2.THRESH_BINARY)
        #img = self.cvb.imgmsg_to_cv2(imgmsg)
        edges = cv2.Canny(r_binary, 100, 200, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi/180, 100,200)
        if lines == None:
            self.rawImg.publish(CvBridge().cv2_to_imgmsg(result))
            return False
        for line in lines:
            rho = line[0][0]
            theta = line[0][1]
            if theta>(np.pi/2-np.pi/6) and theta<(np.pi/6+np.pi/2):
                line_count = line_count +1
                x1 = int(rho*np.cos(theta))
                y1 = int(rho*np.sin(theta))
                if x1 == 0:
                    x2 = x1 +500
                    y2 = y1 
                #x2 = int((rho-result.shape[0]*np.sin(theta))/np.cos(theta))
                else:
                    x2 = int(x1+500)
                    y2 = int(y1+((-x1/y1)*500))
                #if x1<0:
                #    x1 = 0
                # x2>640:
                #    x2 = 640
                pt1 = (x1,y1)
                pt2 = (x2,y2)
                cv2.line(edges, pt1, pt2, (255))
                #print(pt1,pt2)
        self.rawImg.publish(CvBridge().cv2_to_imgmsg(edges))
        if line_count > 0:
            print("line detected")
            return True
        else:
            return False 
#img = cv2.imread('2.jpg')
def main():
    cvb = CvBridge()
    rospy.init_node('traffic_light_detection',anonymous = True)
    rawImg = rospy.Publisher('raw',Image,queue_size=1)
    Redlight = rospy.Publisher('redLight',Image,queue_size = 1)
    element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    video = cv2.VideoCapture(1) 
    while video.isOpened():
        ret, frame = video.read()
        if ret:            
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            #img = cv2.imread('./catkin_ws/src/smartcar/scripts/red_sample.jpg')
            red_lower = np.array([0,85,220])
            red_upper = np.array([20,255,255])
            red_mask = cv2.inRange(hsv,red_lower,red_upper)
            red_target = cv2.bitwise_and(frame,frame,mask = red_mask)
            #for i in range(red_mask.shape[1]):
            #    for j in range(red_mask.shape[0]):
            #        if red_target[j][i][0] + red_target[j][i][1]+red_target[j][i][2] != 0 :
            #            red_target[j][i] = [10,63,255]
            red_target = cv2.erode(red_target,element)
            red_target = cv2.dilate(red_target,element)
            red_gray = cv2.cvtColor(red_target,cv2.COLOR_BGR2GRAY)
            r_ret,r_binary = cv2.threshold(red_mask,127,255,cv2.THRESH_BINARY)
            r_gray2 = cv2.Canny(r_binary, 100, 200) 
            g = r_gray2[:,:] == 255
            count = len(r_gray2[g])
            print(count)   #count around 500   distance is approximately 1.6m
                           #unuse of hough circle detection
                        
       	    #circles_red=cv2.HoughCircles(r_gray2,cv2.HOUGH_GRADIENT,1,600,param1=255,param2=30,minRadius=15,maxRadius=100)
            #if(circles_red is None):
            #    redLight = 0
            #else:
            #    redLight = 1
            #    print("redlight detected!!")
            #rawImg.publish(cvb.cv2_to_imgmsg(frame))
            if count>500:
                 redLight = 0
            else:
                 redLight = 1
            Redlight.publish(cvb.cv2_to_imgmsg(r_gray2))       

if __name__ == "__main__":
    try:
        detector = trafficLightDetector()
        rospy.spin()   #防止程序退出，保证主程序一直循环
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()
'''
if __name__ =="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()
'''
def line_detection(img):
        result = img.copy()
        result = result[340:480,:]
        line_count = 0
        result = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
        #img = self.cvb.imgmsg_to_cv2(imgmsg)
        edges = cv2.Canny(result, 50, 150, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi/180, 100,200)
        if lines == None:
            rawImg.publish(CvBridge().cv2_to_imgmsg(result))
            return False
        for line in lines:
            rho = line[0][0]
            theta = line[0][1]
            if theta>(np.pi/2-np.pi/18) and theta<(np.pi/18+np.pi/2):
                line_count = line_count +1
                x1 = int(rho*np.cos(theta))
                y1 = int(rho*np.sin(theta))
                if x1 == 0:
                    x2 = x1 +500
                    y2 = y1 
                #x2 = int((rho-result.shape[0]*np.sin(theta))/np.cos(theta))
                else:
                    x2 = int(x1+500)
                    y2 = int(y1+((-x1/y1)*500))
                #if x1<0:
                #    x1 = 0
                #if x2>640:
                #    x2 = 640
                pt1 = (x1,y1)
                pt2 = ;(x2,y2)
                cv2.line(result, pt1, pt2, (255))
                #print(pt1,pt2)
        rawImg.publish(CvBridge().cv2_to_imgmsg(result))
        if line_count > 1:
            return True
        else:
            return False 
'''
if __name__ =="__main__":
    initial_parameters()
    rospy.init_node('traffic_light_detection',anonymous = True)
    global rawImg
    rawImg = rospy.Publisher('raw',Image,queue_size=1)
    Video = cv2.VideoCapture(1)
    while Video.isOpened():
        ret, img = Video.read()
        if ret:
            img =cv2.resize(img,(640,480))
            undstrt = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
            boo = line_detection(undstrt)
            print(boo)
'''
