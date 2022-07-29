#!/usr/bin/env python3
import numpy as np
import os
import rospy
import time


os.system("clear")
np.set_printoptions(precision=2, suppress=True)

def runTraining(env,agent,V_d,phi,logName,K_ep_max=15):

    agent.vel_d = [V_d,phi,0.0]
    env.createCSV(logName)

    # ============================
    ##          Episode         
    # ============================

    for k_ep in range(0,K_ep_max):

    
        ## PRE-ALLOCATE REWARD VEC AND OBTAIN THETA VALS
        reward_arr = np.zeros(shape=(agent.n_rollouts)) # Array of reward values for training
        theta = agent.get_theta()             # Generate sample policies from distribution

        ## CONVERT AGENT ARRAYS TO LISTS FOR PUBLISHING
        agent.K_ep_list.append(k_ep)

        agent.mu_1_list.append(agent.mu[0,0])
        agent.mu_2_list.append(agent.mu[1,0])

        agent.sigma_1_list.append(agent.sigma[0,0])
        agent.sigma_2_list.append(agent.sigma[1,0])

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

            ## UPDATE EPISODE/ROLLOUT NUMBER
            agent.k_ep = k_ep
            agent.k_run = k_run
            agent.RL_Publish()

            ## INITIALIZE POLICY PARAMETERS: 
            Tau_thr = theta[0, k_run]    # Tau threshold 10*[s]
            My = theta[1, k_run]         # Policy Moment Action [N*mm]

            ## DOMAIN RANDOMIZATION (UPDATE INERTIA VALUES)
            env.Iyy = rospy.get_param("Iyy") + np.random.normal(0,1.5e-6)
            env.mass = rospy.get_param("/CF_Mass") + np.random.normal(0,0.0005)
            env.updateInertia()

            env.ParamOptim_reset()
            env.startLogging(logName)
            obs,reward,done,info = env.ParamOptim_Flight(Tau_thr/10,My,V_d,phi)
            
            ## ADD VALID REWARD TO TRAINING ARRAY
            reward_arr[k_run] = reward

            ## PUBLISH ROLLOUT DATA
            agent.policy = [Tau_thr,My,0]
            agent.reward = reward
            agent.error_str = env.error_str

            agent.K_run_list.append(k_ep)
            agent.reward_list.append(reward)

            agent.RL_Publish()


            env.capLogging(logName)
            
     

        ## =======  EPISODE COMPLETED  ======= ##
        print(f"Episode # {k_ep:d} training, average reward {np.mean(reward_arr):.3f}")
        agent.train(theta,reward_arr)

        ## PUBLISH AVERAGE REWARD DATA
        agent.Kep_list_reward_avg.append(k_ep)
        agent.reward_avg_list.append(np.mean(reward_arr))
        agent.reward_avg = np.mean(reward_arr)
        agent.RL_Publish()

        if all(agent.sigma < 0.05):
            break

    # ============================
    ##          Episode         
    # ============================

    for k_ep in range(K_ep_max,K_ep_max+3):

    
        ## PRE-ALLOCATE REWARD VEC AND OBTAIN THETA VALS
        reward_arr = np.zeros(shape=(agent.n_rollouts)) # Array of reward values for training
        theta = agent.get_theta()             # Generate sample policies from distribution

        ## CONVERT AGENT ARRAYS TO LISTS FOR PUBLISHING
        agent.K_ep_list.append(k_ep)

        agent.mu_1_list.append(agent.mu[0,0])
        agent.mu_2_list.append(agent.mu[1,0])

        agent.sigma_1_list.append(agent.sigma[0,0])
        agent.sigma_2_list.append(agent.sigma[1,0])

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

            ## UPDATE EPISODE/ROLLOUT NUMBER
            agent.k_ep = k_ep
            agent.k_run = k_run
            agent.RL_Publish()

            ## INITIALIZE POLICY PARAMETERS: 
            Tau_thr = theta[0, k_run]    # Tau threshold 10*[s]
            My = theta[1, k_run]         # Policy Moment Action [N*mm]

            ## DOMAIN RANDOMIZATION (UPDATE INERTIA VALUES)
            env.Iyy = rospy.get_param("Iyy") + np.random.normal(0,1.5e-6)
            env.mass = rospy.get_param("/CF_Mass") + np.random.normal(0,0.0005)
            env.updateInertia()

            env.ParamOptim_reset()
            env.startLogging(logName)
            obs,reward,done,info = env.ParamOptim_Flight(Tau_thr/10,My,V_d,phi)
            
            ## ADD VALID REWARD TO TRAINING ARRAY
            reward_arr[k_run] = reward

            ## PUBLISH ROLLOUT DATA
            agent.policy = [Tau_thr,My,0]
            agent.reward = reward
            agent.error_str = env.error_str

            agent.K_run_list.append(k_ep)
            agent.reward_list.append(reward)

            agent.RL_Publish()


            env.capLogging(logName)
        
        ## PUBLISH AVERAGE REWARD DATA
        agent.Kep_list_reward_avg.append(k_ep)
        agent.reward_avg_list.append(np.mean(reward_arr))
        agent.reward_avg = np.mean(reward_arr)
        agent.RL_Publish()


if __name__ == '__main__':
    from CrazyflieEnv_ParamOpt import CrazyflieEnv_ParamOpt
    from RL_agents.EPHE_Agent import EPHE_Agent

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv_ParamOpt(GZ_Timeout=False)

    ## INIT LEARNING AGENT
    # Mu_Tau value is multiplied by 10 so complete policy is more normalized
    mu_0 = np.array([2.5, 5])       # Initial mu starting point
    sig_0 = np.array([0.5, 1.5])   # Initial sigma starting point
    agent = EPHE_Agent(mu_0,sig_0,n_rollouts=6)


    # ============================
    ##     FLIGHT CONDITIONS  
    # ============================

    ## CONSTANT VELOCITY LAUNCH CONDITIONS
    V_d = 2.5 # [m/s]
    phi = 60   # [deg]

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    logName = f"{agent.agent_type}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}--{env.modelInitials()}.csv"

    runTraining(env,agent,V_d,phi,logName)


    



