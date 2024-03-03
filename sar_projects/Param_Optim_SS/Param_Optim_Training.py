#!/usr/bin/env python3
import numpy as np
import os
import time


os.system("clear")
np.set_printoptions(precision=2, suppress=True)

def runTraining(env,agent,Vel_mag_B_P,Vel_angle_B_P,Plane_Angle,logName,K_ep_max=15):

    # agent.V_mag_rel = V_mag_rel
    # agent.V_angle_rel = V_angle_rel
    # agent.Plane_angle = Plane_Angle
    env.createCSV(logName)

    # ============================
    ##          Episode         
    # ============================

    for K_ep in range(0,K_ep_max):

        ## PRE-ALLOCATE REWARD VEC AND OBTAIN THETA VALS
        reward_arr = np.zeros(shape=(agent.n_rollouts)) # Array of reward values for training
        theta = agent.get_theta()             # Generate sample policies from distribution

        a_Trg_list = 0.5 * (theta[0,:] + 1) * (env.TauThr_range[1] - env.TauThr_range[0]) + env.TauThr_range[0]
        a_Rot_list = 0.5 * (theta[1,:] + 1) * (env.Ang_Acc_range[1] - env.Ang_Acc_range[0]) + env.Ang_Acc_range[0]

        ## CONVERT AGENT ARRAYS TO LISTS FOR PUBLISHING
        agent.K_ep_list.append(K_ep)

        agent.mu_1_list.append(agent.mu[0,0])
        agent.mu_2_list.append(agent.mu[1,0])

        agent.sigma_1_list.append(agent.sigma[0,0])
        agent.sigma_2_list.append(agent.sigma[1,0])

        ## PRINT EPISODE DATA
        print("=============================================")
        print("STARTING Episode # %d" %K_ep)
        print("=============================================")

        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
        print(f"mu_0 = {agent.mu[0,0]:.3f}, \t sig_0 = {agent.sigma[0,0]:.3f}")
        print(f"mu_1 = {agent.mu[1,0]:.3f}, \t sig_1 = {agent.sigma[1,0]:.3f}")
        print('\n')
        

        print("Theta_i = ")
        print(theta[0,:])
        print(theta[1,:])

        print("Policy: ")
        print("a_Trg: ", a_Trg_list)
        print("a_Rot: ", np.round(a_Rot_list,0))
        print("=============================================")


        # ============================
        ##          Run 
        # ============================
        for K_run in range(0,agent.n_rollouts):

            ## UPDATE EPISODE/ROLLOUT NUMBER
            agent.K_ep = K_ep
            agent.K_run = K_run
            agent.RL_Publish()

            ## INITIALIZE POLICY PARAMETERS: 
            a_Trg = a_Trg_list[K_run]  
            a_Rot = a_Rot_list[K_run] 

            print(f"Rollout # {K_run:d} Policy: a_Trg = {a_Trg:.3f}, a_Rot = {a_Rot:.3f}")
            env.reset(V_mag=Vel_mag_B_P,V_angle=Vel_angle_B_P,Plane_Angle=Plane_Angle)
            env.startLogging(logName)

            
            Done = False
            while not Done:
                
                action = env.action_space.sample() # obs gets passed in here
                action[0] = a_Trg
                action[1] = a_Rot

                obs,reward,terminated,truncated,_ = env.step(action)
                Done = terminated or truncated
            
            ## ADD VALID REWARD TO TRAINING ARRAY
            reward_arr[K_run] = reward

            ## PUBLISH ROLLOUT DATA
            agent.policy = [a_Trg,a_Rot]
            agent.reward = reward
            agent.reward_vals = env.reward_vals
            agent.error_str = env.error_str

            agent.K_run_list.append(K_ep)
            agent.reward_list.append(reward)

            agent.RL_Publish()


            env.capLogging(logName)
            
     

        ## =======  EPISODE COMPLETED  ======= ##
        print(f"Episode # {K_ep:d} training, average reward {np.mean(reward_arr):.3f}")
        agent.train(theta,reward_arr)

        ## PUBLISH AVERAGE REWARD DATA
        agent.Kep_list_reward_avg.append(K_ep)
        agent.reward_avg_list.append(np.mean(reward_arr))
        agent.reward_avg = np.mean(reward_arr)
        agent.RL_Publish()




if __name__ == '__main__':
    from Envs.SAR_ParamOpt_Env import SAR_ParamOpt_Sim_SS
    from Agents.EPHE_Agent import EPHE_Agent

    ## INIT GAZEBO ENVIRONMENT
    env = SAR_ParamOpt_Sim_SS(GZ_Timeout=False)

    ## INIT LEARNING AGENT
    # Mu_Tau value is multiplied by 10 so complete policy is more normalized
    mu_0 =  np.array([0.0, 0.0])       # Initial mu starting point
    sig_0 = np.array([0.4, 0.4])   # Initial sigma starting point
    agent = EPHE_Agent(mu_0,sig_0,n_rollouts=8)


    # ============================
    ##     FLIGHT CONDITIONS  
    # ============================

    ## CONSTANT VELOCITY LAUNCH CONDITIONS
    V_mag = 2.5     # [m/s]
    V_angle = 60    # [deg]
    Plane_angle = 0 # [deg]
    env.setAngAcc_range([-100,100])
    env.TauThr_range = [0,0.5]

    ## INITIALIALIZE LOGGING DATA
    trial_num = 25
    logName = f"{agent.agent_type}--trial_{int(trial_num):02d}--{env.SAR_Config}.csv"

    runTraining(env,agent,V_mag,V_angle,Plane_angle,logName,K_ep_max=25)


    



