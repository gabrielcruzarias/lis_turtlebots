#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Twist
from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib
import math
import time
import random
#from turtlebot_2turtles_communication import *
from SimpleServer import *
from multinavigator import *


class WaiterTimings(Waiter):
    timings = {("room1", "room2") : [], ("room1", "room3") : [], ("room1", "kitchen") : [], ("room2", "room1") : [], ("room2", "room3") : [], ("room2", "kitchen") : [], ("room3", "room1") : [], ("room3", "room2") : [], ("room3", "kitchen") : [], ("kitchen", "room1") : [], ("kitchen", "room2") : [], ("kitchen", "room3") : []}
    possible_actions = {"kitchen" : ["GO_TO_ROOM1", "GO_TO_ROOM2", "GO_TO_ROOM3"], "room1" : ["GO_TO_ROOM2", "GO_TO_ROOM3", "GO_TO_KITCHEN"], "room2" : ["GO_TO_ROOM1", "GO_TO_ROOM3", "GO_TO_KITCHEN"], "room3" : ["GO_TO_ROOM1", "GO_TO_ROOM2", "GO_TO_KITCHEN"]}
    
    def __init__(self, name, start_location = "kitchen", start_drinks_ordered = {"room1" : [], "room2" : [], "room3" : []}, start_action = "GO_TO_ROOM1", debug = False, default_velocity = 0.3, default_angular_velocity = 0.75):
        Waiter.__init__(name, start_location, start_drinks_ordered, start_action, debug, default_velocity, default_angular_velocity)
        
    
    def gatherTimings(self):
        while True:
            t = time.time()
            loc = self.location
            if (self.action == "GO_TO_ROOM1"):
                self.goToRoom1()
            elif (self.action == "GO_TO_ROOM2"):
                self.goToRoom2()
            elif (self.action == "GO_TO_ROOM3"):
                self.goToRoom3()
            elif (self.action == "GO_TO_KITCHEN"):
                self.goToKitchen()
            
            self.addTiming(t, loc)
            action_index = random.randint(0, len(self.possible_actions[self.location]) - 1)
            self.action = self.possible_actions[self.location][action_index]
        
    def addTiming(self, t, loc):
        delta_t = time.time() - t
        timings[(loc, self.location)].append(delta_t)
        
    
    
