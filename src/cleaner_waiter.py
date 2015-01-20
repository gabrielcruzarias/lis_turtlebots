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
#from turtlebot_2turtles_communication import *
from SimpleServer import *
from multinavigator import *
from waiter_locations import *
from BeerBotDomain import *

class Waiter(MultiNavigator):
    waiter_ports = {"donatello" : 12346, "leonardo" : 12347}
    DRINKS_ORDERS_LIMIT = {"room1" : 2, "room2" : 2, "room3" : 2}
    # states = "GO_TO_ROOM1", "GO_TO_ROOM2", "GO_TO_ROOM3", "GO_TO_KITCHEN", "WAIT_IN_KITCHEN", "ASK_FOR_DRINK", "GET_DRINK"
    LOCATION_MAPPING = {"room1" : LOC.R1, "room2" : LOC.R2, "room3" : LOC.R3, "kitchen" : LOC.KITCHEN, "after_pr2" : LOC.KITCHEN}
    PR2_MSG_MAPPING = {"not_serving" : PR2.NOT_READY, "drink_in_hand" : PR2.INHAND, "serving_turtlebot" : PR2.SOMEONE, "no_obs" : PR2.NO_OBS}
    
    def __init__(self, name, start_location = "kitchen", start_drinks_ordered = {"room1" : [], "room2" : [], "room3" : []}, start_action = {"donatello" : "GO_TO_ROOM1", "leonardo" : "GO_TO_ROOM2"}, debug = False, default_velocity = 0.3, default_angular_velocity = 0.75):
        MultiNavigator.__init__(self, name, debug, default_velocity, default_angular_velocity)
        #self.state = (location, drinks_ordered, drinks_on_turtlebot, state_of_pr2, state_of_other_turtlebot)
        
        self.action = start_action[name]
        self.location = start_location # room1, room2, room3, kitchen, pr2, after_pr2
        
        self.drinks_ordered = start_drinks_ordered # 3-Tuple<List<2-Tuple<Integer, Float>>>
        self.drinks_on_turtlebot = 0
        
        self.talk_to_pr2_server = SimpleServer(port = self.waiter_ports[name], threading = True)
        pr2_host = "pr2mm1.csail.mit.edu"
        #if (self.debug):
        #    pr2_host = "localhost"
        self.listen_to_pr2_client = SimpleClient(host = pr2_host, port = 12345) # "pr2mm1.csail.mit.edu"
        
        #self.goToPose(ROOM1_HALLWAY[0], ROOM1_HALLWAY[1])
        
        #if (self.name == "donatello"):
        #    self.goToPose(KITCHEN1[0], KITCHEN1[1])
        #else:
        #    self.goToPose(KITCHEN2[0], KITCHEN2[1])

        #self.eventLoop()

    def eventLoop(self):
        rospy.loginfo("starting event loop! current action = " + self.action)
        while True:
            if (self.action == "GO_TO_ROOM1"):
                self.goToRoom1()
                self.action = "GO_TO_KITCHEN"
                #self.transitionToNextState(obs) # GO_TO_KITCHEN, GO_TO_ROOM
            elif (self.action == "GO_TO_ROOM2"):
                self.goToRoom2()
                self.action = "GO_TO_KITCHEN"
                #self.transitionToNextState(obs) # GO_TO_KITCHEN, GO_TO_ROOM
            elif (self.action == "GO_TO_ROOM3"):
                self.goToRoom3()
                self.action = "GO_TO_KITCHEN"
                #self.transitionToNextState(obs) # GO_TO_KITCHEN, GO_TO_ROOM
            elif (self.action == "GO_TO_KITCHEN"):
                self.goToKitchen()
                self.action = "GET_DRINK"
                #self.transitionToNextState(obs) # WAIT_IN_KITCHEN, GO_TO_ROOM
            elif (self.action == "WAIT_IN_KITCHEN"):
                obs = self.waitInKitchen()
                if (obs == "come " + self.name):
                    self.action = "GET_DRINK"
                else:
                    self.action = "GO_TO_ROOM2"
                #self.transitionToNextState(obs) # GET_DRINK, GO_TO_ROOM
                #self.wait_until_msg_is("pr2 ready to place can")
                #self.send_msg_to_pr2("can i come")
                #self.wait_until_msg_is("come " + self.name)
                #self.state = self.APPROACHING_PR2
            elif (self.action == "GET_DRINK"):
                self.getDrink()
                #self.action = "GO_TO_ROOM1"
                self.action = "GO_TO_KITCHEN"
                #self.transitionToNextState(obs) # GO_TO_ROOM
                #self.approach()
                #self.send_msg_to_pr2("turtle in place position")
                #self.state = self.WAITING_FOR_DRINK
            #if self.state == self.WAITING_FOR_DRINK:
                #self.wait_until_msg_is("pr2 placed object")
                #self.move(0.5, 0.25)
                #self.send_msg_to_pr2("turtle left pr2")
                #self.state = self.GOING_TO_KITCHEN

    def transitionToNextAction(self, obs):
        pass

    #####################################################################################
    ################################### MACRO-ACTIONS ###################################
    #####################################################################################
    
    def goToRoom1(self, ignore_drinks = False):
        # initial position = room1, room2, room3, kitchen, or pr2
        rospy.loginfo("Going to room1...")
        if (not self.debug):
            if (self.location == "room1"):
                self.turn(math.pi / 2)
            elif (self.location == "kitchen" or self.location == "room2" or self.location == "room3"):
                self.turn(5/6. * math.pi)
            self.wayposeNavigation(PATH_WAYPOSES[(self.location, "room1")])
        #else:
        #    raw_input("Hit enter to go to room 1...")
        self.location = "room1"
        
        if (ignore_drinks):
            return
        
        self.deliverDrinks()
        rospy.loginfo("STATE = (self.drinks_ordered = " + str(self.drinks_ordered) + " ; self.drinks_on_turtlebot = " + str(self.drinks_on_turtlebot))
        num_drinks = self.getOrders()
        rospy.loginfo("STATE = (self.drinks_ordered = " + str(self.drinks_ordered) + " ; self.drinks_on_turtlebot = " + str(self.drinks_on_turtlebot))
        return (self.LOCATION_MAPPING[self.location], num_drinks, self.drinks_on_turtlebot, self.PR2_MSG_MAPPING["no_obs"])
        
        
    def goToRoom2(self, ignore_drinks = False):
        # initial position = room1, room2, room3, kitchen, or pr2
        rospy.loginfo("Going to room2...")
        if (not self.debug):
            if (self.location == "room1"):
                self.turn(math.pi / 2)
            elif (self.location == "kitchen" or self.location == "room2" or self.location == "room3"):
                self.turn(5/6. * math.pi)
            self.wayposeNavigation(PATH_WAYPOSES[(self.location, "room2")])
        #else:
        #    raw_input("Hit enter to go to room 2...")
        self.location = "room2"
        
        if (ignore_drinks):
            return
        
        self.deliverDrinks()
        rospy.loginfo("STATE = (self.drinks_ordered = " + str(self.drinks_ordered) + " ; self.drinks_on_turtlebot = " + str(self.drinks_on_turtlebot))
        num_drinks = self.getOrders()
        rospy.loginfo("STATE = (self.drinks_ordered = " + str(self.drinks_ordered) + " ; self.drinks_on_turtlebot = " + str(self.drinks_on_turtlebot))
        return (self.LOCATION_MAPPING[self.location], num_drinks, self.drinks_on_turtlebot, self.PR2_MSG_MAPPING["no_obs"])
        
    
    
    def goToRoom3(self, ignore_drinks = False):
        # initial position = room1, room2, room3, kitchen, or pr2
        rospy.loginfo("Going to room3...")
        if (not self.debug):
            if (self.location == "room1"):
                self.turn(math.pi / 2)
            elif (self.location == "kitchen" or self.location == "room2" or self.location == "room3"):
                self.turn(5/6. * math.pi)
            self.wayposeNavigation(PATH_WAYPOSES[(self.location, "room3")])
        #else:
        #    raw_input("Hit enter to go to room 3...")
        self.location = "room3"
        
        if (ignore_drinks):
            return
        
        self.deliverDrinks()
        rospy.loginfo("STATE = (self.drinks_ordered = " + str(self.drinks_ordered) + " ; self.drinks_on_turtlebot = " + str(self.drinks_on_turtlebot))
        num_drinks = self.getOrders()
        rospy.loginfo("STATE = (self.drinks_ordered = " + str(self.drinks_ordered) + " ; self.drinks_on_turtlebot = " + str(self.drinks_on_turtlebot))
        return (self.LOCATION_MAPPING[self.location], num_drinks, self.drinks_on_turtlebot, self.PR2_MSG_MAPPING["no_obs"])
        
    
    def goToKitchen(self, ignore_drinks = False):
        # initial position = room1, room2, room3, kitchen, or pr2
        if (not self.debug):
            if (self.location == "room1"):
                self.turn(math.pi / 2)
            elif (self.location == "room2" or self.location == "room3"):
                self.turn(5/6. * math.pi)
            
            if (self.name == "donatello"):
                self.wayposeNavigation(PATH_WAYPOSES[(self.location, "kitchen1")])
            elif (self.name == "leonardo"):
                self.wayposeNavigation(PATH_WAYPOSES[(self.location, "kitchen2")])
        #else:
        #    raw_input("Hit enter to go to the kitchen...")
        self.location = "kitchen"
        
        if (ignore_drinks):
            return
        
        (pr2_msg, pr2_msg_extra) = self.listenToPR2().split(":")
        print (pr2_msg, pr2_msg_extra)
        if (pr2_msg_extra == " waiting_for_turtlebot"):
            pr2_msg = "drink_in_hand"
        return (self.LOCATION_MAPPING[self.location], 0, self.drinks_on_turtlebot, self.PR2_MSG_MAPPING[pr2_msg])
    
    def waitInKitchen(self):
        #self.wait_until_msg_is("pr2 ready to place can")
        self.send_msg_to_pr2("can i come")
        return self.wait_until_msg_is("serving_turtlebot: come " + self.name)
        
    
    def getDrink(self):
        self.waitInKitchen()
        
        if (not self.debug):
            self.approach()
            self.turn()
        else:
            raw_input("Hit enter to approach PR2...")
        
        self.location = "pr2"
        self.send_msg_to_pr2("turtle in place position")
        self.wait_until_msg_is("serving_turtlebot: pr2 placed object")
        self.drinks_on_turtlebot += 1
        rospy.loginfo("STATE = (self.drinks_ordered = " + str(self.drinks_ordered) + " ; self.drinks_on_turtlebot = " + str(self.drinks_on_turtlebot))
        if (not self.debug):
            self.move(0.5, 0.25)
        else:
            raw_input("Hit enter to leave PR2...")
            
        self.send_msg_to_pr2("turtle left pr2")
        
        if (not self.debug):
            self.wayposeNavigation(PATH_WAYPOSES[(self.location, "after_pr2")])
        self.location = "after_pr2"
        
        (pr2_msg, pr2_msg_extra) = self.listenToPR2().split(":")
        print (pr2_msg, pr2_msg_extra)
        if (pr2_msg_extra == " waiting_for_turtlebot"):
            pr2_msg = "drink_in_hand"
        
        return (self.LOCATION_MAPPING[self.location], 0, self.drinks_on_turtlebot, self.PR2_MSG_MAPPING[pr2_msg])
        
        
    def goToAfterPR2(self):
        self.wayposeNavigation(PATH_WAYPOSES[(self.location, "after_pr2")])
        self.location = "after_pr2"
    
    
    ######################################################################################
    ################################## HELPER FUNCTIONS ##################################
    ######################################################################################
    
    def listenToPR2(self):
        old_msg = None
        msg = self.receive_msg_from_pr2()
        while (old_msg != msg):
            old_msg = msg
            msg = self.receive_msg_from_pr2()
        return msg
        
    ## self.drinks_ordered = [("roomX", number_of_drinks), ...]
    ## self.drinks_on_turtlebot = 0, 1, or 2
    
    #------------------------------------------------------
    # Integer ROOM := usually 0 (deliver drinks to the room that's been waiting the longest), but 
    #
    # CASES:
    #   self.drinks_on_turtlebot < self.drinks_ordered[ROOM][1]
    #       self.drinks_ordered[ROOM][1] -= self.drinks_on_turtlebot
    #       self.drinks_on_turtlebot = 0
    #   self.drinks_on_turtlebot == self.drinks_ordered[ROOM][1]
    #       self.drinks_ordered.pop(ROOM)
    #       self.drinks_on_turtlebot = 0
    #   self.drinks_on_turtlebot > self.drinks_ordered[ROOM][1]
    #       self.drinks_on_turtlebot -= self.drinks_ordered[ROOM][1]
    #       self.drinks_ordered.pop(ROOM)
    #
    #------------------------------------------------------
    def deliverDrinks(self):
        if (len(self.drinks_ordered[self.location]) > 0 and self.drinks_on_turtlebot > 0):
            drinks_to_deliver = min(self.drinks_ordered[self.location][0][0], self.drinks_on_turtlebot)
            raw_input("Grab " + str(drinks_to_deliver) + " drinks.")
        
            # reward
        
            if (self.drinks_on_turtlebot < self.drinks_ordered[self.location][0][0]):
                self.drinks_ordered[self.location][0][0] -= self.drinks_on_turtlebot
                self.drinks_on_turtlebot = 0
            elif (self.drinks_on_turtlebot == self.drinks_ordered[self.location][0][0]):
                self.drinks_ordered[self.location].pop(0)
                self.drinks_on_turtlebot = 0
            elif (self.drinks_on_turtlebot > self.drinks_ordered[self.location][0][0]):
                self.drinks_on_turtlebot -= self.drinks_ordered[self.location][0][0]
                self.drinks_ordered[self.location].pop(0)
            # Receive message saying that the drinks have been picked up by the people
            #drinks_delivered = 1
            #room_number = int(self.location[-1]) # self.location = "roomX" -> X = int(self.location[-1])
            #self.drinks_ordered[room_number] = max(0, self.drinks_ordered[room_number] - drinks_delivered)
        
    def getOrders(self):
        # Receive message saying how many drinks were ordered
        while (True):
            try:
                ordered_drinks = int(raw_input("How many drinks do you want? "))
                break
            except:
                pass
        open_drinks = sum([x[0] for x in self.drinks_ordered[self.location]])
        drinks_to_add = min(self.DRINKS_ORDERS_LIMIT[self.location] - open_drinks, ordered_drinks)
        if (drinks_to_add > 0):
            self.drinks_ordered[self.location].append([drinks_to_add, time.time()])
        return ordered_drinks
        #room_number = int(self.location[-1]) # self.location = "roomX" -> X = int(self.location[-1])
        #self.drinks_ordered[room_number] += 1
    
    def wait_until_msg_is(self, correct_msg):
        rospy.loginfo("waiting to receive following msg from PR2: %s" % correct_msg)
        msg = self.receive_msg_from_pr2()
        while (msg[:len(correct_msg)] != correct_msg):
            msg = self.receive_msg_from_pr2()
        return msg
              
    def send_msg_to_pr2(self, msg):
        rospy.loginfo( "sending message: %s " % msg)
        self.talk_to_pr2_server.update_broadcast(self.name + "," + msg)
        rospy.loginfo( "message sent: %s " % msg)
  
    def receive_msg_from_pr2(self):
        msg_received = False
        while not msg_received:
            try:
                msg = self.listen_to_pr2_client.get_message()
                msg_received = True
                #rospy.loginfo("message received is: %s " % msg)
            except:
                rospy.sleep(1)
        return msg
    
    
    

if __name__=="__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser("select which turtle to use.  default is Donatello")
    parser.add_argument("-d", "--donatello", action="store_true", help="select donatello",dest="donatello")
    parser.add_argument("-l", "--leonardo", action="store_true", help="select leonardo", dest="leonardo")
    args = parser.parse_args()
    if not (args.leonardo or args.donatello):
        name = "donatello"
    elif args.leonardo:
        name = "leonardo"
    else:
        name = "donatello"
    rospy.init_node("waiter_"+name)
    rospy.loginfo("starting waiter with turtle %s " % name)
    waiter = Waiter(name)
    raw_input("Hit enter to start...")
    waiter.eventLoop()
    



    
    








