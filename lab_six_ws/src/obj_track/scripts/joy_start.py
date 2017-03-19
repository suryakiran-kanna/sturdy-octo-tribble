#!/usr/bin/env python

# /blueooth_teleop/joy
# sensor_msgs/Joy
#
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32[] axes
# int32[] buttons
#
# axes: [ lft - l/r, lft - up/down, L2 (1/-1), rgt - l/r, rgt - u/d, R2 (1/-1)]
# buttons: [ x, circle, sq, tri, L1, R1, share, options, play, L3, R3, DL, DR, DU, DD]
#

import rospy
import roslaunch
import sys
import time
import os
from sensor_msgs.msg import Joy

class joy_control(object):

    def __init__(self):

        rate = rospy.Rate(5)
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.joy_callback)

        self.trigger = False
        
        
        rospy.loginfo("In start")

        while not rospy.is_shutdown():
            if (self.trigger == True):
                
                package = 'obj_track'
                executable = 'track.py'
                if self.track == 1:
               	    rospy.loginfo("Starting tracking node")
    	            #node = roslaunch.core.Node(package, executable)
                    #launch = roslaunch.scriptapi.ROSLaunch()
                    #launch.start()
                    #track_process = launch.launch(node)
                if self.track == 0:
               	    rospy.loginfo("Stopping tracking node")
               	    #track_process.stop()
	    self.trigger=False

    def joy_callback(self, data):

        x, circ, sq, tri, L1, R1, share, options, p4, L3, R3, DL, DR, DU, DD = data.buttons
        llr, lud, L2, rlr, rud, R2 = data.axes

	if (circ == 1): 
            rospy.loginfo("Turning on tracking")
            self.trigger = True
	    self.track = 1
	if (x == 1):
            rospy.loginfo("Turning off tracking")
	    self.trigger = False
	    self.track = 0

if __name__ == "__main__":

    try:
        rospy.init_node("joy_start", anonymous=False)
        run = joy_control()  #read in joystick input
    except rospy.ROSInterruptException:
        rospy.loginfo("joy_start node terminated.")
