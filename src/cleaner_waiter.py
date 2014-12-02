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
from basic_turtlebot import *


class Waiter(MultiNavigator):
    OFFICE1_POSITION = {"donatello" : (0, 0), "leonardo" : (0, 0)}
    OFFICE1_ORIENTATION = {"donatello" : (0, 1), "leonardo" : (0, 1)}

    OFFICE2_POSITION = {"donatello" : (0, 0), "leonardo" : (0, 0)}
    OFFICE2_ORIENTATION = {"donatello" : (0, 1), "leonardo" : (0, 1)}

    KITCHEN_POSITION = {"donatello" : (1.79, -0.57), "leonardo" : (1.25, -1.03)}
    KITCHEN_ORIENTATION = {"donatello" : (0.9, 0.44), "leonardo" : (0.78, 0.63)}

    PR2_POSITION = {"donatello" : (0.83, 0.44), "leonardo" : (0, 0)}
    PR2_ORIENTATION = {"donatello" : (-0.63, 0.77), "leonardo" : (0, 1)}
    
    def __init__(self, name, other_turtlebots, start_state, default_velocity = 0.3, default_angular_velocity = 0.75):
        Navigator.__init__(self, name, other_turtlebots, default_velocity, default_angular_velocity)
        self.start_state = start_state
        
