import numpy as np
import time,os
import rospy
from crazyflie_msgs.msg import ImpactData,CtrlData
import math
import pandas as pd


import sys
cwd = os.getcwd()
sys.path.insert(0, f'{cwd}/')


# from crazyflie_rl.src.Crazyflie_env import CrazyflieEnv
from crazyflie_rl.src.Crazyflie_env import CrazyflieEnv
from rospy.exceptions import ROSException

os.system("clear")
np.set_printoptions(precision=2, suppress=True)



if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    # env.launch_dashboard()