#!/usr/bin/env python3


import numpy as np
import pandas as pd
import time,os


from crazyflie_env import CrazyflieEnv
from CF_training import runTraining
from rl_syspepg import rlsysPEPGAgent_reactive,rlsysPEPGAgent_adaptive
from rl_EM import rlEM_PEPGAgent,rlEM_AdaptiveAgent
from rl_cma import CMA_basic,CMA,CMA_sym

os.system("clear")
np.set_printoptions(precision=2, suppress=True)




if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv()
    # env.launch_dashboard()

    print("Environment done")
    ## Home Test List
    df = pd.read_csv("~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/Home_Test_List.csv")
    ## Laptop Test List
    # df = pd.read_csv("~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/Laptop_Test_List.csv")
    arr = df.to_numpy()

    for vz_d,vx_d,trial_num in arr:
        if np.isnan(vz_d):
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
        while True:
            mu = np.random.uniform(1.0,7.0,size=(3,1))
            print(mu)
            if(mu[1]<mu[2]):
                continue
            else:
                break
        sigma = np.array([[2.0],[2.0],[2.0]]) # Initial estimates of sigma: 

        
        ## SIM PARAMETERS
        env.n_rollouts = 10
        env.gamma = 0.95
        env.h_ceiling = 2.5 # [m]

        ## LEARNING AGENT
        agent = rlEM_PEPGAgent(mu,sigma,n_rollouts=env.n_rollouts)
        


        
        ## INITIAL LOGGING DATA
        env.agent_name = agent.agent_type
        env.trial_name = f"{env.agent_name}--Vz_{vz_d:.2f}--Vx_{vx_d:.2f}--trial_{int(trial_num)}"
        
        env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
        env.logging_flag = True


        ## BROKEN ROTOR FIX
        if trial_num %1 == 0:    # There's an issue where rotors detach randomly and prevent model from flipping
            env.relaunch_sim()   # this should help remedy that and make sure it doesn't go on forever
        

        try:
            ## RUN TRIAL
            env.RL_Publish() # Publish data to rl_data topic
            runTraining(env,agent,vx_d,vz_d,k_epMax=20)

        except: ## IF SIM EXCEPTION RAISED THEN CONTINUE BACK TO TRY BLOCK UNTIL SUCCESSFUL COMPLETION
            continue
 
 