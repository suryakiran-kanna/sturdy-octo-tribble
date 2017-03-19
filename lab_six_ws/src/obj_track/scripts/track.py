#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np
from collections import deque
from geometry_msgs.msg import Twist
import argparse
import time
import math
from sensor_msgs.msg import Joy

# linear accleration and decceleration
def smooth_vel(vel_before, vel_final, t_before, t_final, rate):
    rospy.loginfo('in smooth_vel')
    print vel_before, vel_final, t_before, t_final, rate
    step = rate*(t_final-t_before)
    sign = 1.0 if (vel_final > vel_before) else -1.0
    error =	math.fabs(vel_final - vel_before)
    rospy.loginfo('in smooth_vel: error %.2d'%error)
    if error < step:
        rospy.loginfo('vel_final %.2d'%vel_final)
        return vel_final
    else:
        step_vel = vel_before + sign * step
        rospy.loginfo('stepped vel: %.2d'%step_vel)
        return vel_before + sign * step

class Tracker:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window1", 1)
        cv2.namedWindow("window2", 1)
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.joy_callback)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.twist = Twist()
        self.init_size=0.0
        self.track = 0

        self.count = 0
        self.inital_time = time.time()
        self.rate_step = 0.015
        self.ang_vel_before = 0.0



    # joy stick feature
    def joy_callback(self, data):

        x, circ, sq, tri, L1, R1, share, options, p4, L3, R3, DL, DR, DU, DD = data.buttons
        llr, lud, L2, rlr, rud, R2 = data.axes

	if (circ == 1):
            rospy.loginfo("Turning on tracking")
	    self.track = 1
	if (x == 1):
            rospy.loginfo("Turning off tracking")
	    self.track = 0


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

        if (self.track == 1) :
            frameCenter = width/2
            rospy.loginfo ('frameCenter: %.2d' %frameCenter)
            maxleftHysThresh = (frameCenter - 0.10 * frameCenter)-frameCenter
            maxrightHysThresh = (frameCenter + 0.10 * frameCenter)-frameCenter
            rospy.loginfo('maxleftHysThresh: %.2d'%maxleftHysThresh)
            rospy.loginfo('maxrightHysThresh: %.2d'% maxrightHysThresh)
            contours, hierarchy = cv2.findContours(gray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                x,y,w,h = cv2.boundingRect(cnt)
                x=x+w/2
                x=x-width/2
                size=float(max(w,h))
                if self.init_size==0:
                    self.init_size=size
                    size=size/self.init_size
                print x,y,w,h
                print size

                #Control Code Here
                rospy.loginfo('Xerror: %.2d'%x)

                if x > maxrightHysThresh or x < maxleftHysThresh:
                    self.count= self.count + 1
                    if self.count > 5:
                         #rotate
                         self.count = 0

                         rospy.loginfo('Proportional Gain: %.2d'%x)
                         tgt_prop_vel = -float(x)/200
                         rospy.loginfo('tgt_prop_vel: %.2f'%tgt_prop_vel)
                         rospy.loginfo('ang_vel_before: %.2f'% self.ang_vel_before)

                         if not (tgt_prop_vel ==  self.ang_vel_before):
                             rospy.loginfo('In smooth vel if');
                             ang_vel = smooth_vel(self.ang_vel_before, tgt_prop_vel, self.inital_time, time.time(), self.rate_step )
                             self.twist.angular.z = ang_vel
                             self.ang_vel_before = ang_vel
                             self.cmd_vel_pub.publish(self.twist)
                             rospy.loginfo('ang_vel_before before next iteration: %.2f'% self.ang_vel_before)
                         #else:
                            #tgt_prop_vel == ang_vel_before
                            #self.cmd_vel_pub.publish(self.twist)

                        #  if x <= rightXError or x>=leftXError:
                        #      rospy.loginfo('In ang vel 0 if')
                        #      self.twist.angular.z = 0
                        #      self.cmd_vel_pub.publish(self.twist)
                        #def smooth_vel(vel_before, vel_final, t_before, t_final, rate):

        cv2.imshow("window1", image)
        cv2.imshow("window2", masked)
        cv2.waitKey(3)

rospy.init_node('Track_Marker')
Track_Marker = Tracker()
rospy.spin()
