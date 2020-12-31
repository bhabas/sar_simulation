#!/usr/bin/env python3

import numpy as np
import time,os,getpass
from scipy.spatial.transform import Rotation
import threading


from crazyflie_env import CrazyflieEnv
from rl_syspepg import rlsysPEPGAgent_reactive,rlsysPEPGAgent_adaptive
from rl_EM import rlEM_PEPGAgent,rlEM_AdaptiveAgent
from rl_cma import CMA_basic,CMA,CMA_sym

os.system("clear")
np.set_printoptions(precision=2, suppress=True)



# ============================
##     Sim Initialization 
# ============================

## INIT GAZEBO ENVIRONMENT
env = CrazyflieEnv()
env.reset_pos() # Reset Gazebo pos
# env.launch_dashboard()
print("Environment done")








def runTrial(vx_d,vz_d):
    # ============================
    ##          Episode         
    # ============================
    for k_ep in range(0,500):
        env.k_ep = k_ep

        env.mu = agent.mu.flatten().tolist()
        env.sigma = agent.sigma.flatten().tolist()
        env.alpha_mu = agent.alpha_mu.flatten().tolist()
        env.alpha_sigma = agent.alpha_sigma.flatten().tolist()




        mu = agent.mu # Mean for Gaussian distribution
        sigma = agent.sigma # Standard Deviation for Gaussian distribution


        reward = np.zeros(shape=(agent.n_rollouts,1))
        theta_rl,epsilon_rl = agent.get_theta()


        print("=============================================")
        print("STARTING Episode # %d" %k_ep)
        print("=============================================")

        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )

        print("RREV=%.3f, \t theta1=%.3f, \t theta2=%.3f, \t theta3=%.3f" %(mu[0], mu[1], mu[2], 0))
        print("sig1=%.3f, \t sig2=%.3f, \t sig12=%.3f, \t sig2=%.3f," %(sigma[0], sigma[1], sigma[2], 0))
        print('\n')

        print("theta_rl = ")
        print(theta_rl[0,:], "--> RREV")
        print(theta_rl[1,:], "--> Gain_RREV")
        print(theta_rl[1,:], "--> Gain_OF_y")




        # ============================
        ##          Run 
        # ============================
        k_run = 0 # Reset run counter each episode
        while k_run < agent.n_rollouts:
            env.k_run = k_run


            ## RESET TO INITIAL STATE
            
            env.step('home',ctrl_flag=1) # Reset control vals and functionality to default vals
            time.sleep(1.0) # Time for CF to settle
            


            ## INITIALIZE RUN PARAMETERS
            RREV_trigger = theta_rl[0, k_run] # FoV expansion velocity [rad/s]
            G1 = theta_rl[1, k_run]
            G2 = theta_rl[2, k_run]
            policy = theta_rl[:,k_run]
            env.policy = policy.flatten().tolist()
        
            # vz_d = np.random.uniform(low=2.5, high=3.0)
            # vx_d = np.random.uniform(low=-2.0, high=2.0)
            vy_d = 0 
            env.vel_d = [vx_d,vy_d,vz_d] # [m/s]


            ## INIT RUN FLAGS
            t_step = 0
            z_max = 0 # [m]
            
            env.runComplete_flag = False
            env.flip_flag = False

            start_time_rollout = env.getTime()
            start_time_pitch = None
            state_history = None
            repeat_run= False
            error_str = ""


            print("\n!-------------------Episode # %d run # %d-----------------!" %(k_ep,k_run))
            print("RREV: %.3f \t Gain_1: %.3f \t Gain_2: %.3f \t Gain_3: %.3f" %(RREV_trigger, G1, G2, 0))
            print("Vx_d: %.3f \t Vy_d: %.3f \t Vz_d: %.3f" %(vx_d, vy_d, vz_d))


            # ============================
            ##          Rollout 
            # ============================

            env.step('pos',ctrl_flag=0) # Turn off pos control
            env.step('vel',env.vel_d,ctrl_flag=1) # Set desired vel
            
            
            while True:
                
                ## DEFINE CURRENT STATE
                state = np.array(env.state_current)
                
                position = state[1:4] # [x,y,z]
                orientation_q = state[4:8] # Orientation in quat format
                vel = state[8:11] # [vx,vy,vz]
                vx,vy,vz = vel
                omega = state[11:14] # [wx,wy,wz]
                d = h_ceiling - position[2] # Vertical distance of drone from ceiling

                ## ORIENTATION DATA FROM STATE
                qw,qx,qy,qz = orientation_q
                R = Rotation.from_quat([qx,qy,qz,qw])
                R = R.as_matrix() # [b1,b2,b3] Body vectors

                RREV,OF_x,OF_y = vz/d, -vy/d, -vx/d # OF_x,y are estimated optical flow vals assuming no body rotation

                
                # ============================
                ##    Pitch Criteria 
                # ============================
                if (RREV > RREV_trigger) and (env.flip_flag == False):
                    start_time_pitch = env.getTime()
                    env.enableSticky(1)

                    omega_yd = (G1*RREV - G2*abs(OF_y))*np.sign(OF_y)
                    omega_xd = 0.0
                    omega_zd = 0.0

                    env.omega_d = [omega_xd,omega_yd,omega_zd]

                    print('----- pitch starts -----')
                    print('vx=%.3f, vy=%.3f, vz=%.3f' %(vx,vy,vz))
                    print('RREV=%.3f, OF_y=%.3f, OF_x=%.3f, Omega_yd=%.3f' %(RREV, OF_y, OF_x, omega_yd) )   
                    print("Pitch Time: %.3f" %start_time_pitch)

                    ## Start rotation and mark rotation as triggered
                    env.step('omega',env.omega_d,ctrl_flag=1) # Set desired ang. vel 
                    env.flip_flag = True

                # ============================
                ##      Record Keeping  
                # ============================
                ## Keep record of state vector every 10 time steps
                state = state[:, np.newaxis] # Convert [13,] array to [13,1] array
                
                if state_history is None:
                    state_history = state 
                else:
                    if t_step%10==0: # Append state_history columns with current state vector 
                        state_history = np.append(state_history, state, axis=1)
                        env.RL_Publish()


                # ============================
                ##    Termination Criteria 
                # ============================

                # If time since triggered pitch exceeds [0.7s]   
                if env.flip_flag and ((env.getTime()-start_time_pitch) > (0.7)):
                    # I don't like this error formatting, feel free to improve on
                    error_1 = "Rollout Completed: Pitch Timeout"
                    error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_pitch,(env.getTime()-start_time_pitch))
                    print(error_1 + "\n" + error_2)

                    error_str = error_1 + error_2
                    env.runComplete_flag = True

                # If time since run start exceeds [2.5s]
                if (env.getTime() - start_time_rollout) > (2.5):
                    error_1 = "Rollout Completed: Time Exceeded"
                    error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_rollout,(env.getTime()-start_time_rollout))
                    print(error_1 + "\n" + error_2)

                    error_str = error_1 + error_2
                    env.runComplete_flag = True

                # If position falls below max height (There is a lag w/ this)
                z_max = max(position[2],z_max)
                if position[2] <= 0.85*z_max:
                    error_1 = "Rollout Completed: Falling Drone"
                    print(error_1)

                    error_str  = error_1
                    env.runComplete_flag = True
        
                    
                

                # ============================
                ##          Errors  
                # ============================

                ## If nan is found in state vector repeat sim run
                if any(np.isnan(state)): # gazebo sim becomes unstable, relaunch simulation
                    print("NAN found in state vector")
                    error_str = "Error: NAN found in state vector"
                    repeat_run = True
                    break





                if env.runComplete_flag==True:

                    env.step('stop')
                    env.reset_pos()
                    env.step('home',ctrl_flag=1)
                    reward[k_run] = agent.calculate_reward(state_history,h_ceiling)
                    env.reward = reward[k_run]
                    print("Reward = %.3f" %(reward[k_run]))
                    print("!------------------------End Run------------------------! \n")                    
                    break

                t_step += 1
            
            ## =======  RUN COMPLETED  ======= ##

            if repeat_run == True:
                env.relaunch_sim()
            
            else:
                ## UPDATE PUBLISHED REWARD VARIABLES

                env.reward = reward[k_run,0]
                env.RL_Publish()
                k_run += 1 # Move on to next run

            
        ## =======  EPISODE COMPLETED  ======= ##
        print("Episode # %d training, average reward %.3f" %(k_ep, np.mean(reward)))
        agent.train(theta_rl,reward,epsilon_rl)
    
        
    
## =======  MAX TRIALS COMPLETED  ======= ##


if __name__ == '__main__':

    env.n_rollouts = 10
    env.gamma = 0.95
    env.logging_flag = True


    ## SIM PARAMETERS
    h_ceiling = 2.0 # [m]


    # ============================
    ##      Learning Agents
    # ============================

    ## Learning rate
    alpha_mu = np.array([[0.1]])
    alpha_sigma = np.array([[0.05]])

    ## Initial parameters for gaussian function
    mu = np.array([[6.0],[6.0],[6.0]])# Initial estimates of mu: 
    sigma = np.array([[1.5],[1.5],[1.5]]) # Initial estimates of sigma: 



    # agent = rlsysPEPGAgent_reactive(alpha_mu, alpha_sigma, mu,sigma, gamma=0.95,n_rollouts=6)
    # agent = rlsysPEPGAgent_adaptive(alpha_mu,alpha_sigma,mu,sigma,n_rollouts=6)
    agent = rlEM_PEPGAgent(mu,sigma,n_rollouts=6)
    # agent = rlEM_AdaptiveAgent(mu,sigma,n_rollouts=6) # Not working


    

    vx_d = 1.0
    vz_d = 3.0
    trial_num = 1
    env.trial_name = f"Vz_{vz_d}--Vx_{vx_d}--trial_{trial_num}.csv"
    env.agent = "EM_PEPG"

    while True:
        try:
            runTrial(vx_d,vz_d)
            env.relaunch_sim()
        except:
            continue
        break
    



