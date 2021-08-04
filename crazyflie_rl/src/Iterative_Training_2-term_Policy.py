#!/usr/bin/env python3


import numpy as np
import pandas as pd
import time,os
import rospy

from Crazyflie_env import CrazyflieEnv
from CF_training_2term_Policy import runTraining
from RL_agents.rl_EM import rlEM_PEPGAgent


os.system("clear")
np.set_printoptions(precision=2, suppress=True)




if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=True)
    # env.launch_dashboard()

    print("Environment done")
    ## Home Test List
    df = pd.read_csv("~/catkin_ws/src/crazyflie_simulation/crazyflie_data/data_collection/MasterTestList.csv")
    ## Laptop Test List
    # df = pd.read_csv("~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/Laptop_Test_List.csv")
    arr = df.to_numpy()

    for V_d,phi,trial_num in arr:
        if np.isnan(V_d):
            print("Trials are over")
            break

        # ============================
        ##          AGENT  
        # ============================
       
        ## GAUSSIAN DISTRIBUTION PARAMETERS 
        mu_1 = np.random.uniform(2.5,6.5) # RREV typically starts around in this range
        mu_2 = np.random.uniform(3.0,8.0) # My can typically start in this range and climb higher too


        mu = np.array([[mu_1],[mu_2]])  # Initial mu starting point     
        # sigma = np.array([[0.00001],[0.00001]]) # Initial estimates of sigma: 
        # mu = np.array([[6],[2.0]])   
        sigma = np.array([[2.0],[2.0]]) # Initial estimates of sigma: 
        

        ## LEARNING AGENT AND PARAMETERS
        env.n_rollouts = 16
        K_EP_MAX = rospy.get_param("K_EP_MAX")
        # agent = EPHE_Agent(mu,sigma,n_rollouts=env.n_rollouts)
        agent = rlEM_PEPGAgent(mu,sigma,n_rollouts=env.n_rollouts)

        

        ## INITIALIZE LOGGING DATA
        env.agent_name = agent.agent_type
        env.trial_name = f"{env.agent_name}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}"        
        env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
        env.logging_flag = True


        ## RUN TRIAL
        # env.RL_Publish() # Publish data to rl_data topic
        # env.launch_dashboard()
        runTraining(env,agent,V_d,phi,k_epMax=K_EP_MAX)
        env.relaunch_sim()


 
 