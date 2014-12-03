#!/usr/bin/env python

#import roslib
import rospy
from geometry_msgs.msg import Twist
#import sys, select, termios, tty
import time
import math
#import actionlib
#import tf


class Turtlebot(object):
    def __init__(self, default_velocity = 0.3, default_angular_velocity = 0.75):
        self.default_velocity = default_velocity
        self.default_angular_velocity = default_angular_velocity
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
        self.twist = Twist()

    def move(self, distance, velocity = None):
        if (velocity == None):
            velocity = self.default_velocity
        t = time.time()
        while (time.time() - t < distance / float(velocity)):
            self.twist.linear.x = velocity; self.twist.linear.y = 0; self.twist.linear.z = 0
            self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
            self.pub.publish(self.twist)
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
        self.pub.publish(self.twist)

    # Turns the turtlebot alpha radians
    def turn(self, angle, angular_velocity = None):
        if (angular_velocity == None):
            angular_velocity = self.default_angular_velocity
        t = time.time()
        while (time.time() - t < 2 * (angle / angular_velocity)): # I don't know why but it seems like a velocity of 1 is really ~0.5 rad/s
            self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
            self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = math.copysign(angular_velocity, angle)
            self.pub.publish(self.twist)
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
        self.pub.publish(self.twist)

    def stop(self, stop_time = 2):
        t = time.time()
        while (time.time() - t < stop_time):
            self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
            self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
            self.pub.publish(self.twist)


if __name__=="__main__":
    rospy.init_node('basic_turtlebot')

    turtlebot = Turtlebot()

    while (True):
        command = raw_input("Enter command: ")
        if (command == "move"):
            distance = float(raw_input("Enter distance: "))
            velocity = raw_input("Enter velocity: ")
            if (velocity == ""):
                turtlebot.move(distance)
            else:
                turtlebot.move(distance, float(velocity))
        elif (command == "turn"):
            angle = float(raw_input("Enter angle: "))
            angular_velocity = raw_input("Enter angular velocity: ")
            if (angular_velocity == ""):
                turtlebot.turn(angle)
            else:
                turtlebot.turn(angle, float(angular_velocity))
        elif (command == "stop"):
            stop_time = raw_input("Enter stop time: ")
            if (stop_time == ""):
                turtlebot.stop()
            else:
                turtlebot.stop(float(stop_time))
        elif (command == "show_variables"):
            print "Default velocity =", turtlebot.default_velocity
            print "Default angular velocity =", turtlebot.default_angular_velocity
        elif (command == "end"):
            break




