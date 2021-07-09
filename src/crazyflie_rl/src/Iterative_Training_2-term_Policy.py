#!/usr/bin/env python3


import numpy as np
import pandas as pd
import time,os


from Crazyflie_env import CrazyflieEnv
from CF_training_2term_Policy import runTraining
from rl_EM import rlEM_PEPGAgent,EPHE_Agent


os.system("clear")
np.set_printoptions(precision=2, suppress=True)




if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv()
    # env.launch_dashboard()

    print("Environment done")
    ## Home Test List
    df = pd.read_csv("~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/Master_Test_List.csv")
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

        ## LEARNING RATES
        alpha_mu = np.array([[0.1]])
        alpha_sigma = np.array([[0.05]])

        ## GAUSSIAN DISTRIBUTION PARAMETERS 
        mu_1 = np.random.uniform(1.0,4.5) # RREV typically starts around in this range
        mu_2 = np.random.uniform(3.5,5.0) # My can typically start in this range and climb higher too

        # mu_1 = 3.0
        # mu_2 = 4.5

        mu = np.array([[mu_1],[mu_2]])  # Initial mu starting point     
        # sigma = np.array([[0.00001],[0.00001]]) # Initial estimates of sigma: 
        sigma = np.array([[2.0],[2.0]]) # Initial estimates of sigma: 

        
        ## SIM PARAMETERS
        env.n_rollouts = 8
        env.h_ceiling = 2.5 # [m]

        ## LEARNING AGENT
        # agent = EPHE_Agent(mu,sigma,n_rollouts=env.n_rollouts)
        agent = rlEM_PEPGAgent(mu,sigma,n_rollouts=env.n_rollouts)

        

        ## INITIAL LOGGING DATA
        env.agent_name = agent.agent_type
        env.trial_name = f"{env.agent_name}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}"        
        env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
        env.logging_flag = True


        ## RUN TRIAL
        env.RL_Publish() # Publish data to rl_data topic
        # env.launch_dashboard()
        runTraining(env,agent,V_d,phi,k_epMax=20)


 
 