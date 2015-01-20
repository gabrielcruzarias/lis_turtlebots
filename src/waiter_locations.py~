

from point import *
from orientation import *

#MAP LOCATIONS:

#ROOM 2:

#Position = (x, y) = (-4.41, 3.57)
#Orientation = (z, w) = (0.98, 0.19)

# Goal points

ROOM1 = (Point(-4.31, 0.22), Orientation(0.990, 0.144), False)
ROOM2 = (Point(-3.29, 3.87), Orientation(0.982, 0.189), False)
ROOM3 = (Point(-3.59, 7.07), Orientation(0.954, 0.301), False)
KITCHEN1 = (Point(0, -0.83), Orientation(-0.06, 0.998), True)
KITCHEN2 = (Point(0.05, -0.12), Orientation(-0.239, 0.971), True)
AFTER_PR2 = (Point(0.03, 0.57), Orientation(1, 0), False)

# Waypoints

PR2 = (Point(1.1, -0.49), Orientation(0.801, 0.598), False)

ROOM1_ENTRANCE_KITCHEN = (Point(-1, -0.1), Orientation(1, 0), False)
ROOM1_ENTRANCE_OTHER_ROOMS = (Point(-1.74, 0.3), Orientation(-0.707, 0.707), False)
ROOM1_PRE_EXIT = (Point(-4.37, -0.44), Orientation(0, 1), False)
ROOM1_EXIT_KITCHEN = (Point(-1, -0.1), Orientation(0, 1), False)
ROOM1_EXIT_ROOMS = (Point(-1, -0.1), Orientation(0.707, 0.707), False)
ROOM1_HALLWAY = (Point(-1, -0.1), Orientation(0.707, 0.707))
#ROOM1_SPECIAL = (Point(-2.26, 1.50), Orientation(-0.68, 0.73))
ROOM1_ROOM2 = (Point(-1.4, 1.07), Orientation(0.707, 0.707), True)

ROOM2_ENTRANCE_KITCHEN = (Point(-1.35, 3.15), Orientation(1, 0), True) # Entrance to room2 coming from kitchen
ROOM2_ENTRANCE_ROOM3 = (Point(-1.8, 3.4), Orientation(-0.707, 0.707), False)
ROOM2_EXIT = (Point(-1.95, 2.15), Orientation(-0.707, 0.707), False)
ROOM2_HALLWAY = (Point(-1.35, 3.15), Orientation(0.707, 0.707), False)
#ROOM2_ROOM1 = (Point(-2.26, 1.50), Orientation(-0.68, 0.73))
ROOM2_ROOM3 = (Point(-1.17, 4.44), Orientation(0.707, 0.707), False)

ROOM3_ENTRANCE = (Point(-1.37, 6.48), Orientation(1, 0), True)
ROOM3_EXIT = (Point(-1.85, 5.65), Orientation(-0.707, 0.707), True)
ROOM3_HALLWAY = (Point(-1.21, 6.02), Orientation(0.828, 0.560), False) #x,y = -1.53, 6.77
#ROOM3_ROOM2 = (Point(), Orientation())

# PATHS
PATH_WAYPOSES = {}
PATH_WAYPOSES[("room1", "room2")] = [ROOM1_PRE_EXIT, ROOM1_EXIT_ROOMS, ROOM1_ROOM2, ROOM2_ENTRANCE_KITCHEN, ROOM2]
PATH_WAYPOSES[("room1", "room3")] = [ROOM1_PRE_EXIT, ROOM1_EXIT_ROOMS, ROOM1_ROOM2, ROOM2_HALLWAY, ROOM2_ROOM3, ROOM3_ENTRANCE, ROOM3]
PATH_WAYPOSES[("room1", "kitchen1")] = [ROOM1_PRE_EXIT, ROOM1_EXIT_KITCHEN, KITCHEN1]
PATH_WAYPOSES[("room1", "kitchen2")] = [ROOM1_PRE_EXIT, ROOM1_EXIT_KITCHEN, KITCHEN2]
PATH_WAYPOSES[("room1", "room1")] = []

PATH_WAYPOSES[("room2", "room1")] = [ROOM2_EXIT, ROOM1_ENTRANCE_OTHER_ROOMS, ROOM1]
PATH_WAYPOSES[("room2", "room3")] = [ROOM2_EXIT, ROOM2_HALLWAY, ROOM2_ROOM3, ROOM3_ENTRANCE, ROOM3]
PATH_WAYPOSES[("room2", "kitchen1")] = [ROOM2_EXIT, ROOM1_ENTRANCE_OTHER_ROOMS, ROOM1_EXIT_KITCHEN, KITCHEN1]
PATH_WAYPOSES[("room2", "kitchen2")] = [ROOM2_EXIT, ROOM1_ENTRANCE_OTHER_ROOMS, ROOM1_EXIT_KITCHEN, KITCHEN2]
PATH_WAYPOSES[("room2", "room2")] = []

PATH_WAYPOSES[("room3", "room1")] = [ROOM3_EXIT, ROOM2_ENTRANCE_ROOM3] + PATH_WAYPOSES[("room2", "room1")]
PATH_WAYPOSES[("room3", "room2")] = [ROOM3_EXIT, ROOM2_ENTRANCE_ROOM3, ROOM2]
PATH_WAYPOSES[("room3", "kitchen1")] = [ROOM3_EXIT, ROOM2_ENTRANCE_ROOM3] + PATH_WAYPOSES[("room2", "kitchen1")]
PATH_WAYPOSES[("room3", "kitchen2")] = [ROOM3_EXIT, ROOM2_ENTRANCE_ROOM3] + PATH_WAYPOSES[("room2", "kitchen2")]
PATH_WAYPOSES[("room3", "room3")] = []

PATH_WAYPOSES[("kitchen", "after_pr2")] = [PR2, AFTER_PR2]
PATH_WAYPOSES[("kitchen", "room1")] = [ROOM1_ENTRANCE_KITCHEN, ROOM1]
PATH_WAYPOSES[("kitchen", "room2")] = [ROOM1_HALLWAY, ROOM1_ROOM2, ROOM2_ENTRANCE_KITCHEN, ROOM2]
PATH_WAYPOSES[("kitchen", "room3")] = [ROOM1_HALLWAY, ROOM1_ROOM2, ROOM2_HALLWAY, ROOM2_ROOM3, ROOM3_ENTRANCE, ROOM3]
PATH_WAYPOSES[("kitchen", "kitchen1")] = []
PATH_WAYPOSES[("kitchen", "kitchen2")] = []

PATH_WAYPOSES[("pr2", "after_pr2")] = [AFTER_PR2]

PATH_WAYPOSES[("after_pr2", "room1")] = [ROOM1_ENTRANCE_KITCHEN, ROOM1]
PATH_WAYPOSES[("after_pr2", "room2")] = [ROOM1_ROOM2, ROOM2_ENTRANCE_KITCHEN, ROOM2]
PATH_WAYPOSES[("after_pr2", "room3")] = [ROOM1_ROOM2, ROOM2_HALLWAY, ROOM2_ROOM3, ROOM3_ENTRANCE, ROOM3]
PATH_WAYPOSES[("after_pr2", "kitchen1")] = [KITCHEN1]
PATH_WAYPOSES[("after_pr2", "kitchen2")] = [KITCHEN2]


#ROOM1_POSITION = {"donatello" : Point(0, 0), "leonardo" : Point(0, 0)}
#ROOM1_ORIENTATION = {"donatello" : Orientation(0, 1), "leonardo" : Orientation(0, 1)}

#ROOM2_POSITION = {"donatello" : (0, 0), "leonardo" : (0, 0)}
#ROOM2_ORIENTATION = {"donatello" : Orientation(0, 1), "leonardo" : Orientation(0, 1)}

#KITCHEN_POSITION = {"donatello" : (1.79, -0.57), "leonardo" : (1.25, -1.03)}
#KITCHEN_ORIENTATION = {"donatello" : Orientation(0.9, 0.44), "leonardo" : Orientation(0.78, 0.63)}

#PR2_POSITION = {"donatello" : (0.83, 0.44), "leonardo" : (0, 0)}
#PR2_ORIENTATION = {"donatello" : Orientation(-0.63, 0.77), "leonardo" : Orientation(0, 1)}



