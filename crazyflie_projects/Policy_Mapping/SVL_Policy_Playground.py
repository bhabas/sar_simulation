#!/usr/bin/env python3
import numpy as np
import os
import rospy


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)

from crazyflie_env.src.Crazyflie_env import CrazyflieEnv
from crazyflie_env.src.RL_agents.EPHE_Agent import EPHE_Agent



os.system("clear")
np.set_printoptions(precision=2, suppress=True)

def Execute_SVL_Flight(env,agent,V_d,phi,logName):

    env.ParamOptim_reset()
    env.startLogging(logName)
    obs,reward,done,info = env.ParamOptim_Flight(0.1,1,V_d,phi)
    
    ## PUBLISH ROLLOUT DATA
    agent.policy = [0,0,0]
    agent.reward = reward
    agent.error_str = env.error_str
    agent.RL_Publish()


    env.capLogging(logName)
    agent.RL_Publish()





if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    agent = EPHE_Agent()

    ## CHECK THAT POLICY IS IN RL MODE
    if rospy.get_param('/POLICY_TYPE') != "SVL_POLICY":
        raise Exception("Policy is not set to SVL_Policy, change policy setting in Sim_Settings.yaml")
    
    ## INITIALIALIZE LOGGING DATA
    trial_num = 25
    logName = f"SVL_Playground--trial_{int(trial_num):02d}--{env.modelInitials()}.csv"

    env.createCSV(logName)
    
    while True:

        # ============================
        ##     FLIGHT CONDITIONS  
        # ============================
        vel,phi,alpha = env.userInput("Flight Velocity (V_d,phi,alpha): ",float)
        phi_rad = np.radians(phi)
        agent.vel_d = [vel,phi,0.0]

        ## UPDATE EPISODE/ROLLOUT NUMBER
        agent.RL_Publish()

        Execute_SVL_Flight(env,agent,vel,phi,logName)
        agent.k_ep += 1


    



