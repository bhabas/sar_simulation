#!/usr/bin/env python3
import rospy
import numpy as np
import sys
import os

from sar_msgs.srv import Model_Move,Model_MoveRequest

Model_Move_Service = rospy.ServiceProxy('/Model_Movement',Model_Move)


if __name__ == "__main__":

    D = 0.5
    Vy = 4.0


    ## RESET POSITION AND VELOCITY
    Move_srv = Model_MoveRequest()
    
    Move_srv.Pos_0.x = 2.0 - D - 0.027
    Move_srv.Pos_0.y = -6
    Move_srv.Pos_0.z = 0.0

    Move_srv.Vel_0.x = 0.0
    Move_srv.Vel_0.y = Vy
    Move_srv.Vel_0.z = 0.0

    Move_srv.Accel_0.x = 0.0
    Move_srv.Accel_0.y = 0.0
    Move_srv.Accel_0.z = 0.0

    rospy.wait_for_service('/Model_Movement',timeout=1)
    service = rospy.ServiceProxy('/Model_Movement', Model_Move)
    service(Move_srv)


