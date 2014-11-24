#!/usr/bin/env python

# This script will make the robot push an object to a given point in a map.
# Initial conditions: Robot is within ~1.5 meters of the object.
# Goal: Push the object to a given goal point.

import roslib
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import time
import math
import actionlib
import tf
import smach
import smach_ros


# Approach the object. Ends when the bumper sensor is activated
# Assumes the robot is facing the object and within ~1.5 meters of it
def move(distance, velocity):
    t = time.time()
    while (time.time() - t < distance / float(velocity)):
        twist.linear.x = velocity; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)


if __name__=="__main__":
    rospy.init_node('basic_turtlebot')

    pub = rospy.Publisher('~cmd_vel', Twist)
    twist = Twist()

    while (True):
        distance = float(raw_input("Enter distance: "))
        velocity = float(raw_input("Enter velocity: "))
        move(distance, velocity)
