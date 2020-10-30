#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import matplotlib.pyplot as plt

import os


from crazyflie_env import CrazyflieEnv
from rl_syspepg import rlsysPEPGAgent_reactive ,rlsysPEPGAgent_cov, rlsysPEPGAgent_adaptive


# ============================
##     Sim Initialization 
# ============================


## Initialize the environment
username = getpass.getuser()
env = CrazyflieEnv()
print("Environment done")



## Sim Parameters
ep_start = 0 # Default episode start position
h_ceiling = 1.5 # meters





# ==========================
##           PEPG 
# ============================

## Learning rate
alpha_mu = np.array([[0.2]])#[2.0]] )#,[0.1]])
alpha_sigma = np.array([[0.1]])#, [1.0]])#,[0.05]])

## Initial parameters for gaussian function
mu = np.array([[4.5],[-4.5], [1.5] ])# ,[1.5]])#,[1.5]])   # Initial estimates of mu: 
sigma = np.array([[1.5],[1.5] ,[0.25] ])# ,[0.75]])      # Initial estimates of sigma: 

agent = rlsysPEPGAgent_reactive(alpha_mu, alpha_sigma, mu,sigma, gamma=0.95,n_rollout=2)


# ============================
##          Episode 
# ============================
for k_ep in range(ep_start,1000):

    np.set_printoptions(precision=2, suppress=True)
    done = False

    mu = agent.mu
    sigma = agent.sigma

    reward = np.zeros(shape=(2*agent.n_rollout,1))
    reward[:] = np.nan  # initialize reward to be NaN array, size n_rollout x 1
    theta_rl,epsilon_rl = agent.get_theta()

    # ============================
    ##          Run 
    # ============================
    k_run = 0
    while k_run < 2*agent.n_rollout:

        ## RESET TO INITIAL STATE
        env.step('home',ctrl_flag=1) # Reset control vals and functionality
        state = env.reset_pos() # Reset Gazebo pos
        time.sleep(3.0) # time for CF to settle


        ## INITIATE RUN PARAMETERS
        RREV_trigger = theta_rl[0, k_run] # FOV expansion velocity [rad/s]
        G1 = theta_rl[1, k_run]
        G2 = theta_rl[2, k_run]
        policy = theta_rl[:,k_run]
        policy = policy[:,np.newaxis]
        # policy = np.reshape(policy,(-1,1)) # reshaping for data logging

     
        vz_d = np.random.uniform(low=2.5, high=3.0)
        vx_d = np.random.uniform(low=-2.0, high=2.0)
        vy_d = 0 
        v_d = [vx_d,vy_d,vz_d] # [m/s]
        # try adding policy parameter for roll pitch rate for vy ( roll_rate = gain3*omega_x)


        ## INIT RUN FLAGS
        t_step = 0
        z_max = 0 # [m]
        

        # ============================
        ##          Rollout 
        # ============================
        
        env.step('vel',v_d,ctrl_flag=1) # Set desired vel
        env.step('pos',ctrl_flag=0) # turn off pos control
        
        while True:
                
            time.sleep(5e-4) # Time step size [Not sure if this is needed]
            t_step += 1


            ## DEFINE CURRENT STATE [Can we thread this to get states even when above]
            state = env.state_current
            print(state)
            
            position = state[1:4] # [x,y,z]
            orientation_q = state[4:8] # Orientation in quat format
            vel = state[8:11]
            vx,vy,vz = vel
            omega = state[11:14]
            d = h_ceiling - position[2] # distance of drone from ceiling






            


            
            








        
        
