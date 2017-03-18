#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np
from collections import deque
import argparse


class Tracker:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window1", 1)
        cv2.namedWindow("window2", 1)
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        size=0
        x=0
	y=0
	w=0
	h=0        
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width, channels = image.shape
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        pinkLower = np.array([150, 50, 50], np.uint8)
        pinkUpper = np.array([175, 255, 255], np.uint8)
        #pinkLower = np.array([29, 86, 6], np.uint8)
        #pinkUpper = np.array([64, 255, 255], np.uint8)
        mask = cv2.inRange(hsv, pinkLower, pinkUpper)
        mask = cv2.erode(mask, None, iterations=3)
        mask = cv2.dilate(mask, None, iterations=2)
        masked = cv2.bitwise_and(image, image, mask=mask)
        gray=cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)

        contours, hierarchy = cv2.findContours(gray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    

    
        for cnt in contours:

            x,y,w,h = cv2.boundingRect(cnt)
            x=x+w/2
            x=x-width/2
            size=max(w,h)
            print x,y,w,h
            print size

            
        #Control Code Here  

        cv2.imshow("window1", image)
        cv2.imshow("window2", masked)
        cv2.waitKey(3)

rospy.init_node('Track_Marker')
Track_Marker = Tracker()
rospy.spin()


