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
        self.init_size=0

    def image_callback(self, msg):
        size=0
        offset=0
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width, channels = image.shape
        ymin=width
        ymax=0
        size=0
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        pinkLower = np.array([150, 50, 50], np.uint8)
        pinkUpper = np.array([175, 255, 255], np.uint8)
        mask = cv2.inRange(hsv, pinkLower, pinkUpper)
        mask = cv2.erode(mask, None, iterations=3)
        mask = cv2.dilate(mask, None, iterations=3)
        masked = cv2.bitwise_and(image, image, mask=mask)

        for i in range(0, height-1):
            for j in range(0,width-1):
	        if mask[i,j]==255:
                    size=size+1
	            if j<ymin:
	                ymin=j
                    if j>ymax:
		        ymax=j

        offset=ymin+((ymax-ymin)/2)-width/2
        if self.init_size==0:
            self.init_size=size
        size=size/self.init_size

        cv2.imshow("window1", image)
        cv2.imshow("window2", masked)
        cv2.waitKey(3)

rospy.init_node('Track_Marker')
Track_Marker = Tracker()
rospy.spin()


