#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation

import rospy
from gazebo_communication_pkg.msg import GlobalState # Custom message format








def global_stateSub(): # Subscriber for receiving global state info
    rospy.init_node('dashboard_node')
    rospy.Subscriber('/global_state',GlobalState,global_stateCallback)
    rospy.spin()

def global_stateCallback(data):
    gs_msg = data # gs_msg <= global_state_msg

    ## SET TIME VALUE FROM TOPIC
    t_temp = gs_msg.header.stamp.secs
    ns_temp = gs_msg.header.stamp.nsecs
    t = t_temp+ns_temp*1e-9 # (seconds + nanoseconds)
    
    ## SIMPLIFY STATE VALUES FROM TOPIC
    global_pos = gs_msg.global_pose.position
    global_quat = gs_msg.global_pose.orientation
    global_vel = gs_msg.global_twist.linear
    global_omega = gs_msg.global_twist.angular
    
    if global_quat.w == 0: # If zero at startup set quat.w to one to prevent errors
        global_quat.w = 1

    ## SET STATE VALUES FROM TOPIC
    position = [global_pos.x,global_pos.y,global_pos.z]
    orientation_q = [global_quat.w,global_quat.x,global_quat.y,global_quat.z]
    velocity = [global_vel.x,global_vel.y,global_vel.z]
    omega = [global_omega.x,global_omega.y,global_omega.z]


    ## COMBINE INTO COMPREHENSIVE LIST
    state_current = [t] + position + orientation_q +velocity + omega ## t (float) -> [t] (list)
    print(state_current)

if __name__ == '__main__':
    global_stateSub()