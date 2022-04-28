#!/usr/bin/env python3
import numpy as np
import os
import rospy
import time


from Crazyflie_env import CrazyflieEnv
from RL_agents.rl_EM import rlEM_PEPGAgent
from ExecuteFlight import executeFlight


os.system("clear")
np.set_printoptions(precision=2, suppress=True)


if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    # env.launch_dashboard()
    

    ## INIT LEARNING AGENT
    # Mu_Tau value is multiplied by 10 so complete policy is more normalized
    mu = np.array([[2.5], [5]])       # Initial mu starting point
    sigma = np.array([[0.5],[1.5]])   # Initial sigma starting point

    env.n_rollouts = 8
    agent = rlEM_PEPGAgent(mu,sigma,env.n_rollouts)


    # ============================
    ##     FLIGHT CONDITIONS  
    # ============================

    ## CONSTANT VELOCITY LAUNCH CONDITIONS
    V_d = 2.5 # [m/s]
    phi = 60   # [deg]
    phi_rad = np.radians(phi)
    env.vel_d = [V_d*np.cos(phi_rad), 0.0, V_d*np.sin(phi_rad)] # [m/s]


    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    env.agent_name = agent.agent_type
    env.trial_name = f"{env.agent_name}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}--{env.modelInitials()}"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
    env.Logging_Flag = True
    env.createCSV(env.filepath)


    # ============================
    ##          Episode         
    # ============================

    for k_ep in range(0,rospy.get_param("K_EP_MAX")):

        ## UPDATE EPISODE NUMBER
        env.k_ep = k_ep

        ## CONVERT AGENT ARRAYS TO LISTS FOR PUBLISHING
        env.mu = agent.mu.flatten().tolist()                # Mean for Gaussian distribution
        env.sigma = agent.sigma.flatten().tolist()          # Standard Deviation for Gaussian distribution

        env.mu_1_list.append(env.mu[0])
        env.mu_2_list.append(env.mu[1])

        env.sigma_1_list.append(env.sigma[0])
        env.sigma_2_list.append(env.sigma[1])

        
        ## PRE-ALLOCATE REWARD VEC AND OBTAIN THETA VALS
        training_arr = np.zeros(shape=(agent.n_rollouts,1)) # Array of reward values for training
        theta_rl,epsilon_rl = agent.get_theta()             # Generate sample policies from distribution

        ## PRINT EPISODE DATA
        print("=============================================")
        print("STARTING Episode # %d" %k_ep)
        print("=============================================")

        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
        print(f"mu_0 = {agent.mu[0,0]:.3f}, \t sig_0 = {agent.sigma[0,0]:.3f}")
        print(f"mu_1 = {agent.mu[1,0]:.3f}, \t sig_1 = {agent.sigma[1,0]:.3f}")
        print('\n')
        

        print("theta_rl = ")
        print(theta_rl[0,:], "--> Tau")
        print(theta_rl[1,:], "--> My")

        # ============================
        ##          Run 
        # ============================
        for env.k_run in range(0,env.n_rollouts):
            # input("press enter")


            ## UPDATE RUN NUMBER
            k_run = env.k_run # Local variables are faster to access then class variables
            


            ## INITIALIZE POLICY PARAMETERS: 
            Tau_thr = theta_rl[0, k_run]    # Tau threshold 10*[s]
            My = theta_rl[1, k_run]         # Policy Moment Action [N*mm]
            G2 = 0.0                        # Deprecated policy term
            
            env.policy = [Tau_thr/10,np.abs(My),G2]

            env.RL_Publish()

            try: # Use try block to catch raised exceptions and attempt rollout again
                env.Iyy = rospy.get_param("Iyy") + np.random.normal(0,1.5e-6)
                env.mass = rospy.get_param("/CF_Mass") + np.random.normal(0,0.0005)
                env.updateInertia()
                executeFlight(env,agent)

                if env.repeat_run == True: # Runs when error detected
                    env.relaunch_sim()
                    continue

            except rospy.service.ServiceException:
                continue

            ## ADD VALID REWARD TO TRAINING ARRAY
            training_arr[k_run] = env.reward

            env.reward_list.append(int(env.reward))
            env.reward_avg = training_arr[np.nonzero(training_arr)].mean()

            ## PUBLISH UPDATED REWARD VARIABLES
            env.RL_Publish()

        env.reward_avg_list.append(int(env.reward_avg))
        env.RL_Publish()

        ## =======  EPISODE COMPLETED  ======= ##
        print(f"Episode # {k_ep:d} training, average reward {env.reward_avg:.3f}")
        agent.train(theta_rl,training_arr,epsilon_rl)


