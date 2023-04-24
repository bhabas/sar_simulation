#!/usr/bin/env python3
import numpy as np

import os
import time
import sys
import subprocess
import threading
import signal
import rospy
import gym
from .CrazyflieEnv_Base import CrazyflieEnv_Base

## ROS MESSAGES AND SERVICES
from std_srvs.srv import Empty
from crazyflie_msgs.srv import domainRand,domainRandRequest
from crazyflie_msgs.srv import ModelMove,ModelMoveRequest


from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty,EmptyRequest

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
CYAN = '\033[96m'
ENDC = '\033[m'

class CrazyflieEnv_Sim(CrazyflieEnv_Base,gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self,GZ_Timeout=True,debug_mode=False):
        CrazyflieEnv_Base.__init__(self)
        self.env_name = "CF_Gazebo"
        self.debug_mode = debug_mode

        ## GAZEBO SIMULATION INITIALIZATION            
        rospy.init_node("Crazyflie_Env_Sim_Node")





    
    




if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv_Sim()

    rospy.spin()