#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionResult
from SimpleServer import *
from geometry_msgs.msg import Twist
from ar_track_alvar.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib
import math
#from turtlebot_2turtles_communication import *
from multinavigator import *


class Waiter(MultiNavigator):
    waiter_ports = {"donatello" : 12346, "leonardo" : 12347}
    # states = "GO_TO_ROOM1", "GO_TO_ROOM2", "GO_TO_ROOM3", "GO_TO_KITCHEN", "WAIT_IN_KITCHEN", "ASK_FOR_DRINK", "GET_DRINK"
    def __init__(self, name, start_location = "after_pr2", start_drinks_ordered = [0, 0, 0], start_action = "GO_TO_ROOM1", default_velocity = 0.3, default_angular_velocity = 0.75):
        MultiNavigator.__init__(self, name, default_velocity, default_angular_velocity)
        #self.state = (location, drinks_ordered)
        self.location = start_location # room1, room2, room3, kitchen, pr2, after_pr2
        self.drinks_ordered = start_drinks_ordered # 0, 1, 2
        self.action = start_action
        
        self.server = SimpleServer(port = waiter_ports[name], threading = True)
        
        self.client = SimpleClient(host = "pr2mm1.csail.mit.edu", port = 12345)

    def eventLoop(self, start_state):
        rospy.loginfo("starting event loop!  current state = %d" % self.state)
        while True:
            if (self.action == "GO_TO_ROOM1"):
                self.go_to_room1()
                self.action = "GO_TO_KITCHEN"
                #self.transitionToNextState(obs) # GO_TO_KITCHEN, GO_TO_ROOM
            elif (self.action == "GO_TO_ROOM2"):
                self.go_to_room2()
                self.action = "GO_TO_KITCHEN"
                #self.transitionToNextState(obs) # GO_TO_KITCHEN, GO_TO_ROOM
            elif (self.action == "GO_TO_ROOM3"):
                self.go_to_room3()
                self.action = "GO_TO_KITCHEN"
                #self.transitionToNextState(obs) # GO_TO_KITCHEN, GO_TO_ROOM
            elif (self.action == "GO_TO_KITCHEN"):
                self.go_to_kitchen()
                self.action = "WAIT_IN_KITCHEN"
                #self.transitionToNextState(obs) # WAIT_IN_KITCHEN, GO_TO_ROOM
            elif (self.action == "WAIT_IN_KITCHEN"):
                self.wait_in_kitchen()
                self.action = "GET_DRINK"
                #self.transitionToNextState(obs) # GET_DRINK, GO_TO_ROOM
                #self.wait_until_msg_is("pr2 ready to place can")
                #self.send_msg_to_pr2("can i come")
                #self.wait_until_msg_is("come " + self.name)
                #self.state = self.APPROACHING_PR2
            elif (self.action == "GET_DRINK"):
                self.get_drink()
                self.action = "GO_TO_ROOM3"
                #self.transitionToNextState(obs) # GO_TO_ROOM
                #self.approach()
                #self.send_msg_to_pr2("turtle in place position")
                #self.state = self.WAITING_FOR_DRINK
            #if self.state == self.WAITING_FOR_DRINK:
                #self.wait_until_msg_is("pr2 placed object")
                #self.move(0.5, 0.25)
                #self.send_msg_to_pr2("turtle left pr2")
                #self.state = self.GOING_TO_KITCHEN

    def transitionToNextState(self, obs):
        pass

    #####################################################################################
    ################################### MACRO-ACTIONS ###################################
    #####################################################################################
    
    def go_to_room1(self):
        # initial position = room1, room2, room3, kitchen, or pr2
        self.wayposeNavigation(PATH_WAYPOSES[(self.location, "room1")])
        self.location = "room1"
        self.deliverDrinks()
        self.getOrders()
        
    def go_to_room2(self):
        # initial position = room1, room2, room3, kitchen, or pr2
        self.wayposeNavigation(PATH_WAYPOSES[(self.location, "room2")])
        self.location = "room2"
        self.deliverDrinks()
        self.getOrders()
    
    def go_to_room3(self):
        # initial position = room1, room2, room3, kitchen, or pr2
        self.wayposeNavigation(PATH_WAYPOSES[(self.location, "room3")])
        self.location = "room3"
        self.deliverDrinks()
        self.getOrders()
    
    def go_to_kitchen(self):
        # initial position = room1, room2, room3, kitchen, or pr2
        if (self.name == "donatello"):
            self.wayposeNavigation(PATH_WAYPOSES[(self.location, "kitchen1")])
        elif (self.name == "leonardo"):
            self.wayposeNavigation(PATH_WAYPOSES[(self.location, "kitchen2")])
        self.location = "kitchen"
        self.listenToPR2()
    
    def wait_in_kitchen(self):
        self.wait_until_msg_is("pr2 ready to place can")
    
    def get_drink(self):
        self.approach()
        self.send_msg_to_pr2("turtle in place position")
        self.turn()
        self.wait_until_msg_is("pr2 placed object")
        self.move(0.5, 0.25)
        self.send_msg_to_pr2("turtle left pr2")
    
    
    ######################################################################################
    ################################## HELPER FUNCTIONS ##################################
    ######################################################################################
    
    def listenToPR2(self):
        pass
        
    def deliverDrinks(self):
        # Receive message saying that the drinks have been picked up by the people
        drinks_delivered = 1
        room_number = int(self.location[-1])
        self.drinks_ordered[room_number] = max(0, self.drinks_ordered[room_number] - drinks_delivered)
        
    def getOrders(self):
        # Receive message saying how many drinks were ordered
        ordered_drinks = 1
        room_number = int(self.location[-1])
        self.drinks_ordered[room_number] += 1
    
    def wait_until_msg_is(self, correct_msg):
        rospy.loginfo("waiting to receive following msg from PR2: %s"\
        % correct_msg)
        msg = self.receive_msg_from_pr2()
        while msg != correct_msg:
            msg = self.receive_msg_from_pr2()
              
    def send_msg_to_pr2(self, msg):
        rospy.loginfo( "sending message: %s " % msg)
        self.server.update_broadcast(self.name + "," + msg)
        rospy.loginfo( "message sent: %s " % msg)
  
    def receive_msg_from_pr2(self):
        msg_received = False
        while not msg_received:
            try:
                msg = self.client.get_message()
                msg_received = True
                #rospy.loginfo("message received is: %s " % msg)
            except:
                rospy.sleep(1)
        return msg
    
    
    








