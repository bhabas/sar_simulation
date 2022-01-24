#!/usr/bin/env python3

import numpy as np
import time
import os
import sys
import rospy
import rospkg


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(0,BASE_PATH)

from crazyflie_msgs.msg import ImpactData,CtrlData
from Crazyflie_env import CrazyflieEnv
from RL_agents.rl_EM import rlEM_PEPGAgent
from ExecuteFlight import executeFlight


from rospy.exceptions import ROSException

os.system("clear")
np.set_printoptions(precision=2, suppress=True)



if __name__ == '__main__':
    ## DEFINE FLIGHT CONDITIONS
    num_flights = 20
    vels = np.random.uniform(2.5,3.5,num_flights)
    angles = np.random.uniform(40,90,num_flights)
    print(vels,angles)


    ## INIT GAZEBO ENVIRONMENT
    agent = rlEM_PEPGAgent()
    env = CrazyflieEnv(gazeboTimeout=True)
    env.policy = [0,0,0] # NN policy

    ## INIT LOGGING DATA
    trial_num = 24
    env.trial_name = f"NN_Policy--trial_{int(trial_num):02d}--Model_{env.modelName[10:]}"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
    env.logging_flag = True
    env.create_csv(env.filepath)
    

    ## RUN TRIAL
    env.RL_Publish() # Publish data to rl_data topic
    time.sleep(3)

    for ii in range(num_flights):
        V_d = vels[ii]
        phi_rad = np.radians(angles[ii])
        env.vel_trial = [V_d*np.cos(phi_rad), 0.0, V_d*np.sin(phi_rad)] # [m/s]
        env.k_run = ii

        try:
            executeFlight(env,agent)
        except rospy.service.ServiceException: ## IF SIM EXCEPTION RAISED THEN CONTINUE BACK TO TRY BLOCK UNTIL SUCCESSFUL COMPLETION
                continue