#!/usr/bin/env python

from threading import Thread
from nav_msgs.msg import Odometry
from SimpleServer import SimpleServer, SimpleClient

class MultiNavigator(Navigator):
    ports = {"donatello" : 12348, "leonardo" : 12349}
    hosts = {"donatello" : "10.68.0.171", "leonardo" : "10.68.0.175"}
    DISTANCE_TOLERANCE = 1.0
    def __init__(self, name, other_turtlebots, default_velocity = 0.3, default_angular_velocity = 0.75):
        Navigator.__init__(self, default_velocity, default_angular_velocity)
        self.name = name
        self.position = Point(0, 0)
        self.angle = 0
        
        # Subscribe to /odom topic and broadcast it to the other turtlebots (via SimpleServer)
        odometry_publisher = SimpleServer(port = self.ports[name], threading = True)
        rospy.Subscriber('/odom', Odometry, self.publishOdometry)
        
        # Create listeners for other turtlebot's pose
        odometry_listener = {}
        for turtlebot in other_turtlebots:
            odometry_listener[turtlebot] = SimpleClient(host = self.hosts[name], port = self.ports[name])
        self.t = Thread(target = self.listenToOdometries)
        self.t.start()

    def publishOdometry(self, Odometry):
        position = Odometry.pose.position
        (x, y) = (position.x, position.y)
        angle = getTheta(Odometry.pose.orientation.w, Odometry.pose.orientation.z)
        self.position = Point(x, y)
        self.angle = angle
        odometry_publisher.update_msg(str(x) + "," + str(y) + "," + str(angle))

    def listenToOdometries(self):
        # Only applies when the turtlebot is navigating to a goal
        if (self.going_to_goal):
            close_to_other_turtlebots = False
            while (True):
                for turtlebot in odometry_listener:
                    (x, y, theta) = odometry_listener[turtlebot].get_message().split(",")
                    if (self.position.distance(Point(x, y)) < self.DISTANCE_TOLERANCE):
                        close_to_other_turtlebots = True
        
            if (close_to_other_turtlebots):
                # Tell the turtlebot to stop, so that it can detect the other turtlebot and reroute around it
                self.stop() # Defaults to stopping for 2 seconds
                
                # Give the turtlebot time to move (travel to the goal). Without this, the turtlebot wouldn't move at all.
                time.sleep(10)


# Gets the yaw angle from the quaternion describing the object's position
def getTheta(w, z):
    mag = math.sqrt(w ** 2 + z ** 2)
    w /= mag
    z /= mag
    return math.atan2(2 * w * z, w ** 2 - z ** 2) #- math.pi / 2
