#!/usr/bin/env python3
import numpy as np
import os
import rospy
import time


from Crazyflie_env2 import CrazyflieEnv
from RL_agents.EPHE_Agent import EPHE_Agent


os.system("clear")
np.set_printoptions(precision=2, suppress=True)


if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)

    ## INIT LEARNING AGENT
    # Mu_Tau value is multiplied by 10 so complete policy is more normalized
    mu_0 = np.array([2.5, 5])       # Initial mu starting point
    sig_0 = np.array([0.5, 1.5])   # Initial sigma starting point

    agent = EPHE_Agent(mu_0,sig_0,n_rollouts=8)


    # ============================
    ##     FLIGHT CONDITIONS  
    # ============================

    ## CONSTANT VELOCITY LAUNCH CONDITIONS
    V_d = 2.5 # [m/s]
    phi = 60   # [deg]
    phi_rad = np.radians(phi)

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    trial_name = f"{agent.agent_type}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}--{env.modelInitials()}"
    env.filepath = f"{env.loggingPath}/{trial_name}.csv"
    env.createCSV(env.filepath)


    # ============================
    ##          Episode         
    # ============================

    for k_ep in range(0,15):

        
        # ## CONVERT AGENT ARRAYS TO LISTS FOR PUBLISHING
        # env.mu = agent.mu.flatten().tolist()                # Mean for Gaussian distribution
        # env.sigma = agent.sigma.flatten().tolist()          # Standard Deviation for Gaussian distribution

        # env.mu_1_list.append(env.mu[0])
        # env.mu_2_list.append(env.mu[1])

        # env.sigma_1_list.append(env.sigma[0])
        # env.sigma_2_list.append(env.sigma[1])

        
        ## PRE-ALLOCATE REWARD VEC AND OBTAIN THETA VALS
        reward_arr = np.zeros(shape=(agent.n_rollouts)) # Array of reward values for training
        theta = agent.get_theta()             # Generate sample policies from distribution

        ## PRINT EPISODE DATA
        print("=============================================")
        print("STARTING Episode # %d" %k_ep)
        print("=============================================")

        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
        print(f"mu_0 = {agent.mu[0,0]:.3f}, \t sig_0 = {agent.sigma[0,0]:.3f}")
        print(f"mu_1 = {agent.mu[1,0]:.3f}, \t sig_1 = {agent.sigma[1,0]:.3f}")
        print('\n')
        

        print("theta_i = ")
        print(theta[0,:], "--> Tau")
        print(theta[1,:], "--> My")

        # ============================
        ##          Run 
        # ============================
        for k_run in range(0,agent.n_rollouts):


            ## INITIALIZE POLICY PARAMETERS: 
            Tau_thr = theta[0, k_run]    # Tau threshold 10*[s]
            My = theta[1, k_run]         # Policy Moment Action [N*mm]
            G2 = 0.0                        # Deprecated policy term

            env.reset()
            obs,reward,done,info = env.ParamOptim_Flight(Tau_thr/10,My,V_d,phi)

            ## ADD VALID REWARD TO TRAINING ARRAY
            reward_arr[k_run] = reward
            
     

        ## =======  EPISODE COMPLETED  ======= ##
        print(f"Episode # {k_ep:d} training, average reward {np.mean(reward_arr):.3f}")
        agent.train(theta,reward_arr)


