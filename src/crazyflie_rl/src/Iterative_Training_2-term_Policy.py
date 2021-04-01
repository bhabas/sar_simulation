#!/usr/bin/env python3


import numpy as np
import pandas as pd
import time,os


from Crazyflie_env import CrazyflieEnv
from CF_training_2term_Policy import runTraining
from rl_EM import rlEM_PEPGAgent


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

        ## GAUSSIAN PARAMETERS (CHECK THAT G1 > G2)
        #  System has a hard time learning if G1 < G2
        
        mu = np.random.uniform(1.0,3.5,size=(2,1))
        # mu = np.array([[1.5],[5.536]])                 # Initial mu starting point
        sigma = np.array([[2.5],[2.5]]) # Initial estimates of sigma: 

        
        ## SIM PARAMETERS
        env.n_rollouts = 8
        env.h_ceiling = 3.0 # [m]

        ## LEARNING AGENT
        agent = rlEM_PEPGAgent(mu,sigma,n_rollouts=env.n_rollouts)
        


        
        ## INITIAL LOGGING DATA
        env.agent_name = agent.agent_type
        env.trial_name = f"{env.agent_name}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}"        
        env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
        env.logging_flag = True



        # ## BROKEN ROTOR FIX
        # if trial_num %1 == 0:    # There's an issue where rotors detach randomly and prevent model from flipping
        #     env.relaunch_sim()   # this should help remedy that and make sure it doesn't go on forever
        

        try:
            ## RUN TRIAL
            env.RL_Publish() # Publish data to rl_data topic
            runTraining(env,agent,V_d,phi,k_epMax=25)

        except: ## IF SIM EXCEPTION RAISED THEN CONTINUE BACK TO TRY BLOCK UNTIL SUCCESSFUL COMPLETION
            continue
 
 