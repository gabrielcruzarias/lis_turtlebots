#!/usr/bin/env python

from ar_track_alvar.msg import AlvarMarkers
from basic_turtlebot import *

class TurtlebotWithAR(Turtlebot):
    def __init__(self, default_velocity = 0.3, default_angular_velocity = 0.75):
        Turtlebot.__init__(self, default_velocity, default_angular_velocity)
        self.obj_x = 10.0; self.obj_y = 10.0; self.obj_theta = 0.0
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.objectPose)

    # Gets the pose of the object relative to the robot (through the AR tags)
    def objectPose(self, AlvarMarkers):
        if (len(AlvarMarkers.markers) > 0):
            self.obj_y = AlvarMarkers.markers[0].pose.pose.position.y
            self.obj_x = AlvarMarkers.markers[0].pose.pose.position.x
            orientation = Orientation(0, 0, AlvarMarkers.markers[0].pose.pose.orientation.z, AlvarMarkers.markers[0].pose.pose.orientation.w)
            self.obj_theta = orientation.getTheta()
            #print "y = ", obj_y, ", theta = ", obj_theta / math.pi, "*pi"

    # Face the object
    # INITIAL CONDITIONS: Robot is close to the object
    # GOAL: The robot faces the object and the robot, object, and goal all lie on the same line (the robot can just move straight
    # forward and it'll hit the object and eventually the goal)
    def face(self, error = 0.1):
        self.obj_y = 10.0
        w = 0.4
        start_t = time.time()
        TIME_TO_FAIL = 60 # Check this later, it shouldn't need to be this big
        while (abs(self.obj_y) > abs(error)):
            t = time.time()
            while (abs(self.obj_y) > abs(error)):
                if (time.time() - start_t > TIME_TO_FAIL):
                    return "box_not_found"
                if (abs(self.obj_y) < 0.95):
                    start_t = time.time()
                if (time.time() - t > 6 and abs(self.obj_y) > 0.3):
                    self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
                    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
                    self.pub.publish(self.twist)
                    time.sleep(1.5)
                    t = time.time()
                self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
                self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = math.copysign(max(min(abs(self.obj_y), w), 0.2), self.obj_y)
                self.pub.publish(self.twist)
            self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
            self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
            self.pub.publish(self.twist)
            self.obj_y = math.copysign(10.0, self.obj_y)
            time.sleep(2)
        return "obj_found"


    def approach(self, desired_distance = 0.4):
        distance = math.sqrt(self.obj_x**2 + self.obj_y**2)
        t = time.time()
        while (distance > desired_distance):
            if (abs(self.obj_y) > 0.25):
                face(0.1)
                t = time.time()
            print distance
            v = min(0.3, max(distance, 0.15))
            self.twist.linear.x = v; self.twist.linear.y = 0; self.twist.linear.z = 0
            self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
            self.pub.publish(self.twist)
            time.sleep(0.3)
            distance = math.sqrt(self.obj_x**2 + self.obj_y**2)
        print distance
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
        self.pub.publish(self.twist)





