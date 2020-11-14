#!/usr/bin/env python3

import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from dist_srv_msg.srv import Dist,DistResponse



class DistanceCalculator():

    def __init__(self):
        self.odom = rospy.Subscriber('/odom',Odometry,self.compute_dist) # subscribe to odom topic
        self.total_distance = 0.0
        self.last_pos = Point() # msg type for pure position inside Odometry
        self.first_odom = True

        self.srv = rospy.Service('/get_distance',Dist,self.srvCallback)
        rospy.loginfo('Service is running')

    def compute_dist(self,data):
        p = data.pose.pose.position # Type Point

        if self.first_odom == True: # On first message set initial position
            self.first_odom = False
            self.last_pos = p

        dist =  math.sqrt((p.x - self.last_pos.x)*(p.x - self.last_pos.x) + 
                            (p.y - self.last_pos.y)*(p.y - self.last_pos.y))
        
        self.total_distance = self.total_distance + dist
        self.last_pos = p

    def srvCallback(self,request):
        dist = self.total_distance

        if request.units == "miles":
            dist = dist * 0.00062

        resp = DistResponse()
        resp.dist = dist

        return resp

rospy.init_node('distance_moved')
d = DistanceCalculator()
rospy.spin()

















