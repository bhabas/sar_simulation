#!/usr/bin/env python2
from __future__ import division

import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped, Twist, Vector3
from mavros_msgs.msg import AttitudeTarget
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread, Event
from tf.transformations import quaternion_from_euler


class MavrosOffboard(MavrosTestCommon):
    def setUp(self):
        if not super(MavrosOffboard, self).setUp():
            return False

        self.att = AttitudeTarget()
        self.att_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        self.att_thread_done = True
        #self.att_thread.start()
        
        self.vel = Twist()
        self.vel_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        self.vel_thread = Thread(target=self.send_vel, args=())
        self.vel_thread.daemon = True
        self.vel_thread_done = False
        self.vel_thread.start()     

        self.vel2 = Twist()
        self.att2 = AttitudeTarget()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        self.att.orientation = Quaternion()
        self.att.thrust = 1.2
        self.att.type_mask = 128 # ignore attitude
        return True 


    def tearDown(self):
        super(MavrosOffboard, self).tearDown()


    def send_att(self):
        rate = rospy.Rate(500)  # Hz
        #self.att.body_rate = Vector3()
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"
        #self.att.orientation = Quaternion(*quaternion_from_euler(-0.25, 0.5,
         #                                                        0))
        self.att.orientation = Quaternion()
        self.att.thrust = 1.2
        #self.att.type_mask = 7  # ignore body rate
        self.att.type_mask = 128 # ignore attitude

        while not rospy.is_shutdown() and not self.att_thread_done:
            self.att.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    
    def send_vel(self):
        rate = rospy.Rate(500)  # Hz

        while not rospy.is_shutdown() and not self.vel_thread_done:
            self.vel_setpoint_pub.publish(self.vel)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    '''def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset'''

    def reach_vel(self, vx, vy, vz, p, q, r):
        self.vel.linear = Vector3(vx, vy, vz)

        
    def reach_orientation(self, phi, theta, psi):
        quaternion = quaternion_from_euler(phi, theta, psi)
        self.att.orientation = Quaternion(*quaternion)

    
    def reach_omega(self, p, q, r):
        self.att.body_rate = Vector3(p ,q, r)

    
    def is_at_velocity(self, vel_desired, radius):
        vel_desired = np.array(vel_desired)
        vel_real = self.local_velocity.twist.linear
        vel_real = np.array([vel_real.x, vel_real.y, vel_real.z])

        return np.linalg.norm(vel_desired - vel_real) < radius

    
    def is_at_position(self, pos_desired, radius):
        pos_desired = np.array(pos_desired)
        pos_real = self.local_position.pose.position
        pos_real = np.array([pos_real.x, pos_real.y, pos_real.z])

        return np.linalg.norm(pos_desired - pos_real) < radius


    def publish_velocity(self, vx, vy, vz):
        self.vel2.linear = Vector3(vx, vy, vz)
        self.vel_setpoint_pub.publish(self.vel2)

    def publish_att(self, p, q, r):
        self.att2.header.stamp = rospy.Time.now()
        self.att2.body_rate = Vector3(p ,q, r)
        self.att_setpoint_pub.publish(self.att2)
