#!/usr/bin/env python

import random

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

'''
    This publishes a Marker every 10 secons which has 2 points attached to it.
    The points are redefined randomly every 10 seconds
'''

class obst_pub():

    def __init__(self):
        markerPub = rospy.Publisher('pp_obstacle', Marker, queue_size=10)
        rospy.Subscriber("pp_obstacle", Marker, self.callback)
        rospy.init_node('obst_pub', anonymous=True)

        rate = rospy.Rate(1)

        self.pp_obstacle = Marker()
        self.pp_obstacle.header.frame_id = "path_planner"
        self.pp_obstacle.header.stamp    = rospy.get_rostime()
        self.pp_obstacle.ns = "path_planner"
        self.pp_obstacle.id = 0
        self.pp_obstacle.type = 8 # point
        self.pp_obstacle.action = 0
        self.pp_obstacle.pose.position.x = 0.0
        self.pp_obstacle.pose.position.y = 0.0
        self.pp_obstacle.pose.position.z = 0.0
        self.pp_obstacle.pose.orientation.x = 0
        self.pp_obstacle.pose.orientation.y = 0
        self.pp_obstacle.pose.orientation.z = 0
        self.pp_obstacle.pose.orientation.w = 1.0
        self.pp_obstacle.scale.x = 1.0
        self.pp_obstacle.scale.y = 1.0
        self.pp_obstacle.scale.z = 1.0

        self.pp_obstacle.color.r = 1.0
        self.pp_obstacle.color.g = 1.0
        self.pp_obstacle.color.b = 1.0
        self.pp_obstacle.color.a = 1.0

        self.pp_obstacle.lifetime = rospy.Duration(0)

        start_point = Point()
        start_point.x = 1.0
        start_point.y = 1.0
        start_point.z = 1.0

        self.pp_obstacle.points.append(start_point)

        self.pp_obstacle.text = "0"

        #a counter so the location changes

        self.counterMax = 10; # waiting time = counterMax*rospy.Rate
        self.counter = self.counterMax;

        while not rospy.is_shutdown():

            self.counter = self.counter + 1;

            if (self.counter > self.counterMax):

                markerPub.publish(self.pp_obstacle)

                self.counter = 0

                start_point = Point()
                start_point.x = random.uniform(0, 80)
                start_point.y = random.uniform(0, 80)
                start_point.z = 1.0

		# self.pp_obstacle.text = "update 1"

                self.pp_obstacle.points[0] = start_point


            #print "sending marker", self.pp_obstacle
            rate.sleep()

    def callback(self, data):
        #data is of type Marker
        #lets print some information
        print "data received"
        print "start_point:\t", data.points[0].x, ", ", data.points[0].y, ", ", data.points[0].z
        print "point 1:\t", data.points[0].x+20, ", ", data.points[0].y, ", ", data.points[0].z
	print "point 2:\t", data.points[0].x+20, ", ", data.points[0].y+20, ", ", data.points[0].z
	print "point 3:\t", data.points[0].x, ", ", data.points[0].y+20, ", ", data.points[0].z
        print "text :\t", data.text

obst_pub()
