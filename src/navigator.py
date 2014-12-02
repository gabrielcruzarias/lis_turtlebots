#!/usr/bin/env python

#import roslib
import rospy
from geometry_msgs.msg import Twist
#import sys, select, termios, tty
import time
import math
import actionlib
#import tf
from basic_turtlebot import *

class Navigator(Turtlebot):
    def __init__(self, default_velocity = 0.3, default_angular_velocity = 0.75):
        Turtlebot.__init__(self, default_velocity, default_angular_velocity)
        self.nav = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.nav.wait_for_server()
        self.goal = MoveBaseGoal()
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goalResult)
        self.going_to_goal = False

    def goToPose(self, position, orientation, frame = "map"):
        self.publishGoal(position, orientation, frame)
        return self.waitToReachGoal()

    def publishGoal(self, position, orientation, frame = "map"):
        self.going_to_goal = True
        self.goal.target_pose.header.frame_id = frame
        self.goal.target_pose.header.stamp.secs = rospy.get_time()

        self.goal.target_pose.pose.position.x = position.x; goal.target_pose.pose.position.y = position.y
        self.goal.target_pose.pose.orientation.z = orientation[0]; goal.target_pose.pose.orientation.w = orientation[1]
        self.nav.send_goal(goal)
        self.nav.wait_for_result(rospy.Duration.from_sec(5.0)) # Does this line do anything?

    def waitToReachGoal():
        while (self.going_to_goal):
            rospy.sleep(1)
        return True

    def goalResult(self, MoveBaseActionResult):
        self.going_to_goal = False
        #if (MoveBaseActionResult.status.status == 3):
        #    self.going_to_goal = False
