#!/usr/bin/env python

#import roslib
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
#import sys, select, termios, tty
import time
import math
#import actionlib
#import tf
from orientation import *


class Turtlebot(object):
    def __init__(self, debug = False, default_velocity = 0.3, default_angular_velocity = 1.15):
        self.debug = debug
        self.default_velocity = default_velocity
        self.default_angular_velocity = default_angular_velocity
        if (not debug):
            self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
            self.twist = Twist()
            rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.processSensing)
            self.found_object = False
        
    # Moves the turtlebot "distance" meters at "velocity" m/s
    def move(self, distance, velocity = None):
        if (self.debug):
            raw_input("Hit enter to move " + str(distance) + " meters with velocity of " + str(velocity) + " m/s...")
            return
            
        if (velocity == None):
            velocity = self.default_velocity
        t = time.time()
        while (time.time() - t < distance / float(velocity)):
            self.setVelocity(velocity, 0)
        self.setVelocity(0, 0)

    # Turns the turtlebot "angle" radians at "angular_velocity" rad/s
    def turn(self, angle = math.pi / 2, angular_velocity = None):
        if (self.debug):
            raw_input("Hit enter to turn " + str(angle) + " radians with angular velocity of " + str(angular_velocity) + " rad/s...")
            return
            
        if (angular_velocity == None):
            angular_velocity = self.default_angular_velocity
        t = time.time()
        while (time.time() - t < 2 * (angle / angular_velocity)): # I don't know why but it seems like a velocity of 1 is really ~0.5 rad/s, so I multiply by 2 to fix that problem
            self.setVelocity(0, math.copysign(angular_velocity, angle))
        self.setVelocity(0, 0)

    # Makes the turtlebot stop for "stop_time" seconds
    def stop(self, stop_time = 2):
        t = time.time()
        while (time.time() - t < stop_time):
            self.setVelocity(0, 0)
            
    # Sets the linear velocity of the robot to "linear" m/s and the angular velocity to "angular" rad/s
    def setVelocity(self, linear, angular):
        self.twist.linear.x = linear; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = angular
        self.pub.publish(self.twist)
        
    # Approach the object. Ends when the bumper sensor is activated
    # Assumes the robot is facing the object and within ~1.5 meters of it
    def bumperApproach(self):
        start_t = time.time()
        TIME_TO_FAIL = 30 # This might not have to be this big
        self.found_object = False
        while (not self.found_object):
            if (time.time() - start_t > TIME_TO_FAIL):
                rospy.loginfo("ERROR: box_not_reached")
                break
            self.twist.linear.x = 0.1; self.twist.linear.y = 0; self.twist.linear.z = 0
            self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0 # Look at the object's pose every time to adjust the angular velocity.
            self.pub.publish(self.twist)
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
        self.pub.publish(self.twist)
        #time.sleep(2) # This is just to let me know that the robot is transitioning from approaching the object to pushing it.
        print "Done approaching"
        
    # This tells us when the robot is in contact with the object
    def processSensing(self, BumperEvent):
        if (not self.found_object and BumperEvent.PRESSED == 1):
            self.found_object = True
        elif (self.found_object and BumperEvent.PRESSED == 0):
            self.found_object = False


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
        elif (command == "bumper"):
            turtlebot.bumperApproach()
        elif (command == "end"):
            break




