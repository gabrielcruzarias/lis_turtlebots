

from Point import *
from Orientation import *

#MAP LOCATIONS:

#ROOM 2:

#Position = (x, y) = (-4.41, 3.57)
#Orientation = (z, w) = (0.98, 0.19)

# Goal points

ROOM1 = (Point(), Orientation())
ROOM2 = (Point(), Orientation())
ROOM3 = (Point(), Orientation())
KITCHEN1 = (Point(0, -0.67), Orientation(0, 1))
KITCHEN2 = (Point(0, -0.1), Orientation(-0.196, 0.981))
AFTER_PR2 = (Point(0, 0.74), Orientation(1, 0))

# Waypoints

PR2 = (Point(), Orientation())

ROOM1_ENTRANCE = (Point(), Orientation())
ROOM1_EXIT = (Point(), Orientation())
ROOM1_HALLWAY = (Point(-1.12, 0), Orientation(0, 1))

ROOM2_ENTRANCE = (Point(), Orientation())
ROOM2_EXIT = (Point(), Orientation())
ROOM2_HALLWAY = (Point(), Orientation())

ROOM3_ENTRANCE = (Point(), Orientation())
ROOM3_EXIT = (Point(), Orientation())
ROOM3_HALLWAY = (Point(), Orientation())


# PATHS
PATH_WAYPOSES = {}
PATH_WAYPOSES[("room1", "room2")] = [ROOM1_EXIT, ROOM1_ENTRANCE, ROOM2_HALLWAY, ROOM2_ENTRANCE, ROOM2]
PATH_WAYPOSES[("room1", "room3")] = [ROOM1_EXIT, ROOM1_ENTRANCE, ROOM2_HALLWAY, ROOM3_HALLWAY, ROOM3_ENTRANCE, ROOM3]
PATH_WAYPOSES[("room1", "kitchen1")] = [ROOM1_EXIT, ROOM1_ENTRANCE, KITCHEN1]
PATH_WAYPOSES[("room1", "kitchen2")] = [ROOM1_EXIT, ROOM1_ENTRANCE, KITCHEN2]

PATH_WAYPOSES[("room2", "room1")] = [ROOM2_EXIT, ROOM1_ENTRANCE, ROOM1]
PATH_WAYPOSES[("room2", "room3")] = [ROOM2_EXIT, ROOM2_HALLWAY, ROOM3_HALLWAY, ROOM3_ENTRANCE, ROOM3]
PATH_WAYPOSES[("room2", "kitchen1")] = [ROOM2_EXIT, ROOM1_ENTRANCE, KITCHEN1]
PATH_WAYPOSES[("room2", "kitchen2")] = [ROOM2_EXIT, ROOM1_ENTRANCE, KITCHEN2]

PATH_WAYPOSES[("room3", "room1")] = [ROOM3_EXIT, ROOM2_ENTRANCE, ROOM2_EXIT, ROOM1_ENTRANCE, ROOM1]
PATH_WAYPOSES[("room3", "room2")] = [ROOM3_EXIT, ROOM2_ENTRANCE, ROOM2]
PATH_WAYPOSES[("room3", "kitchen1")] = [ROOM3_EXIT, ROOM2_ENTRANCE, ROOM2_EXIT, ROOM1_ENTRANCE, KITCHEN1]
PATH_WAYPOSES[("room3", "kitchen2")] = [ROOM3_EXIT, ROOM2_ENTRANCE, ROOM2_EXIT, ROOM1_ENTRANCE, KITCHEN2]

PATH_WAYPOSES[("after_pr2", "room1")] = [ROOM1_ENTRANCE, ROOM1]
PATH_WAYPOSES[("after_pr2", "room2")] = [ROOM2_HALLWAY, ROOM2_ENTRANCE, ROOM2]
PATH_WAYPOSES[("after_pr2", "room3")] = [ROOM2_HALLWAY, ROOM3_HALLWAY, ROOM3_ENTRANCE, ROOM3]


#ROOM1_POSITION = {"donatello" : Point(0, 0), "leonardo" : Point(0, 0)}
#ROOM1_ORIENTATION = {"donatello" : Orientation(0, 1), "leonardo" : Orientation(0, 1)}

#ROOM2_POSITION = {"donatello" : (0, 0), "leonardo" : (0, 0)}
#ROOM2_ORIENTATION = {"donatello" : Orientation(0, 1), "leonardo" : Orientation(0, 1)}

#KITCHEN_POSITION = {"donatello" : (1.79, -0.57), "leonardo" : (1.25, -1.03)}
#KITCHEN_ORIENTATION = {"donatello" : Orientation(0.9, 0.44), "leonardo" : Orientation(0.78, 0.63)}

#PR2_POSITION = {"donatello" : (0.83, 0.44), "leonardo" : (0, 0)}
#PR2_ORIENTATION = {"donatello" : Orientation(-0.63, 0.77), "leonardo" : Orientation(0, 1)}



