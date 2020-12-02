#!/usr/bin/env python3

import numpy as np
import time,os,getpass
from multiprocessing import Process,Array,Value
from scipy.spatial.transform import Rotation
import threading


from crazyflie_env import CrazyflieEnv
from rl_syspepg import rlsysPEPGAgent_reactive ,rlsysPEPGAgent_cov, rlsysPEPGAgent_adaptive
from rl_EM import rlEM_PEPGAgent, rlEM_PEPG_CovAgent, rlEM_OutlierAgent , rlEMsys_PEPGAgent,rlEM_AdaptiveCovAgent, rlEM_AdaptiveCovAgent3D, rlEM_AdaptiveAgent
from rl_cma import CMA_basic,CMA,CMA_sym

from utility.dashboard import runGraph

os.system("clear")


def main():

    # ============================
    ##     Sim Initialization 
    # ============================



    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv()
    print("Environment done")

    ## INIT STATE RECIEVING THREAD
    state_thread = threading.Thread(target=env.get_state)
    state_thread.start() # Start thread that continually recieves state array from Gazebo

    ## INIT USER AND DATA RECORDING
    username = getpass.getuser()
    start_time = time.strftime('_%Y-%m-%d_%H:%M:%S', time.localtime(time.time()))
    file_name = '/home/'+username+'/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/log/' + username + start_time + '.csv'
    env.create_csv(file_name,record = False)
    

    ## SIM PARAMETERS
    ep_start = 0 # Default episode start position
    h_ceiling = 1.5 # [m]]


    # ============================
    ##           PEPG 
    # ============================

    ## Learning rate
    alpha_mu = np.array([[0.2]])#[2.0]] )#,[0.1]])
    alpha_sigma = np.array([[0.1]])#, [1.0]])#,[0.05]])

    ## Initial parameters for gaussian function
    mu = np.array([[5.5],[8.9], [1.5] ])# ,[1.5]])#,[1.5]])   # Initial estimates of mu: 
    sigma = np.array([[1.0],[1.0] ,[0.25] ])# ,[0.75]])      # Initial estimates of sigma: 

    # noise tests all started at:
    #mu = np.array([[5.0],[-5.0] ])
    #sigma = np.array([[3.0],[3.0] ])
    #  
    agent = rlsysPEPGAgent_reactive(alpha_mu, alpha_sigma, mu,sigma, gamma=0.95,n_rollout=4)
    #agent = rlsysPEPGAgent_cov(alpha_mu, alpha_sigma, mu,sigma, gamma=0.95,n_rollout=4)
    #agent = rlEM_PEPGAgent(mu,sigma,n_rollout=5)
    #agent = rlsysPEPGAgent_adaptive(alpha_mu,alpha_sigma,mu,sigma,n_rollout=10)
    #agent = rlEM_OutlierAgent(mu,sigma,n_rollout=5) # diverges?
    #agent = rlEM_PEPG_CovAgent(mu,sigma,n_rollout=5)

    #agent = rlEMsys_PEPGAgent(mu,sigma,n_rollout=5)
    #agent = rlEM_AdaptiveCovAgent(mu,sigma,gamma=0.95,n_rollout=5)
    #agent = rlEM_AdaptiveCovAgent3D(mu,sigma,gamma=0.95,n_rollout=5)


    # agent = rlEM_AdaptiveAgent(mu,sigma,n_rollout=5)
    # ============================
    ##           CMA 
    # ============================

    # seems to be unstable if mu is close to zero (coverges to deterimistic)
    ## Initial parameters for gaussian function
    #mu = np.array([[3.0],[-3.0] ])#,[1.5]])   # Initial estimates of mu: 
    #sigma = np.array([[1.0],[1.0] , [0.5]])      # Initial estimates of sigma: 

    # # For CMA_basic, make sure simga has 3 inputs / for pepg it must be 2
    # agent = CMA_basic(mu,sigma,N_best=0.3,n_rollout=10)

    #agent = CMA(n=2,gamma = 0.9) # number of problem dimensions
    #agent = CMA_sym(n=2,gamma=0.9)




    # ============================
    ##          Episode 
    # ============================
    for k_ep in range(ep_start,1000):

        np.set_printoptions(precision=2, suppress=True)
        done = False
        # N_ROLLOUTS.value = agent.n_rollout # Global n_rollout variable

        mu = agent.mu # Mean for Gaussian distribution
        sigma = agent.sigma # Standard Deviation for Gaussian distribution
        omega_d = [0,0,0] # Junk declaration to cleanup warning or possible error


        reward = np.zeros(shape=(2*agent.n_rollout,1))
        reward[:] = np.nan  # Init reward to be NaN array, size n_rollout x 1 [Not sure why?]
        theta_rl,epsilon_rl = agent.get_theta()

        #mu = agent.xmean
        #sigma = np.array([agent.C[0,0],agent.C[1,1],agent.C[0,1]])


        print("=============================================")
        print("STARTING Episode # %d" %k_ep)
        print("=============================================")

        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )

        print("RREV=%.3f, \t theta1=%.3f, \t theta2=%.3f, \t theta3=%.3f" %(mu[0], mu[1], mu[0], mu[0]))
        print("sig1=%.3f, \t sig2=%.3f, \t sig12=%.3f, \t sig2=%.3f," %(sigma[0], sigma[1], sigma[0], sigma[0]))
        print('\n')

        print("theta_rl = ")
        print(theta_rl[0,:], "--> RREV")
        print(theta_rl[1,:], "--> Gain_RREV")




        # ============================
        ##          Run 
        # ============================
        k_run = 0 # Reset run counter each episode
        while k_run < 2*agent.n_rollout:

            ## RESET TO INITIAL STATE
            env.step('home',ctrl_flag=1) # Reset control vals and functionality to default vals
            state = env.reset_pos() # Reset Gazebo pos
            time.sleep(3.0) # Time for CF to settle


            ## INITIALIZE RUN PARAMETERS
            RREV_trigger = theta_rl[0, k_run] # FoV expansion velocity [rad/s]
            G1 = theta_rl[1, k_run]
            G2 = theta_rl[2, k_run]
            policy = theta_rl[:,k_run]
            policy = policy[:,np.newaxis] # reshaping for data logging [ change [3,] -> [3,1] ]
        
            vz_d = np.random.uniform(low=2.5, high=3.0)
            vx_d = np.random.uniform(low=-2.0, high=2.0)
            vy_d = 0 
            vx_d = 0
            v_d = [vx_d,vy_d,vz_d] # [m/s]
            # Note: try adding policy parameter for roll pitch rate for vy ( roll_rate = gain3*omega_x)


            ## INIT RUN FLAGS
            t_step = 0
            z_max = 0 # [m]
            
            done_rollout = False
            start_time_rollout = env.getTime()
            start_time_pitch = None
            pitch_triggered = False
            state_history = None
            repeat_run= False
            error_str = ""


            print("\n!-------------------Episode # %d run # %d-----------------!" %(k_ep,k_run))
            print("RREV: %.3f \t Gain_1: %.3f \t Gain_2: %.3f \t Gain_3: %.3f" %(RREV_trigger, G1, G2, 0))
            print("Vz_d: %.3f \t Vx_d: %.3f \t Vy_d: %.3f" %(vz_d, vx_d, vy_d))


            # ============================
            ##          Rollout 
            # ============================
            
            env.step('vel',v_d,ctrl_flag=1) # Set desired vel
            env.step('pos',ctrl_flag=0) # turn off pos control
            
            while True:
                
                ## DEFINE CURRENT STATE
                state = env.get_state()
                
                position = state[1:4] # [x,y,z]
                orientation_q = state[4:8] # Orientation in quat format
                vel = state[8:11]
                vx,vy,vz = vel
                omega = state[11:14]
                d = h_ceiling - position[2] # distance of drone from ceiling

                ## ORIENTATION DATA FROM STATE
                qw,qx,qy,qz = orientation_q
                R = Rotation.from_quat([qx,qy,qz,qw])
                R = R.as_matrix() # [b1,b2,b3] Body vectors

                RREV, OF_y, OF_x = vz/d, vx/d, vy/d # OF_x,y are estimated optical flow vals assuming no body rotation
                sensor_data = [RREV, OF_y, OF_x] # simplified for data recording
                
                # ============================
                ##    Pitch Criteria 
                # ============================
                if (RREV > RREV_trigger) and (pitch_triggered == False):
                    start_time_pitch = env.getTime()
                    env.enableSticky(1)

                    omega_yd = (G1*RREV + G2*abs(OF_y))*np.sign(OF_y)# + qomega #omega_x#*(1-b3y)#sin(r[1]*3.14159/180)
                    omega_xd = 0.0 #theta_rl[3,k_run] * omega_y
                    omega_zd = 0.0

                    omega_d = [omega_xd,omega_yd,omega_zd]

                    print('----- pitch starts -----')
                    print('vz=%.3f, vx=%.3f, vy=%.3f' %(vz,vx,vy))
                    print('RREV=%.3f, OF_y=%.3f, OF_x=%.3f, Omega_yd=%.3f' %(RREV, OF_y, OF_x, omega_yd) )   
                    print("Pitch Time: %.3f" %start_time_pitch)

                    ## Start rotation and mark rotation as triggered
                    env.step('omega',omega_d,ctrl_flag=1) # Set desired ang. vel 
                    pitch_triggered = True


                # ============================
                ##    Termination Criteria 
                # ============================

                # If time since triggered pitch exceeds [0.7s]   
                if pitch_triggered and ((env.getTime()-start_time_pitch) > (0.7)):
                    # I don't like this error formatting, feel free to improve on
                    error_1 = "Rollout Completed: Pitch Timeout"
                    error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_pitch,(env.getTime()-start_time_pitch))
                    print(error_1 + "\n" + error_2)

                    error_str = error_1 + error_2
                    done_rollout = True

                # If time since rollout start exceeds [2.5s]
                if (env.getTime() - start_time_rollout) > (2.5):
                    error_1 = "Rollout Completed: Time Exceeded"
                    error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_rollout,(env.getTime()-start_time_rollout))
                    print(error_1 + "\n" + error_2)
    
                    error_str = error_1 + error_2
                    done_rollout = True

                # If position falls below max height (There is a lag w/ this)
                z_max = max(position[2],z_max)
                if position[2] <= 0.75*z_max:
                    error_1 = "Rollout Completed: Falling Drone"
                    print(error_1)

                    error_str  = error_1
                    done_rollout = True
                    env.step('stop') # turn motors off before resetting position
                    env.reset_pos()
                    
                

                # ============================
                ##          Errors  
                # ============================

                ## If nan is found in state vector repeat sim run
                if any(np.isnan(state)): # gazebo sim becomes unstable, relaunch simulation
                    print("NAN found in state vector")
                    error_str = "Error: NAN found in state vector"
                    repeat_run = True
                    break

                elif env.timeout:
                    print("Controller reciever thread timed out")
                    error_str = "Error: Controller Timeout"
                    repeat_run = True
                    break
                

                # ============================
                ##      Record Keeping  
                # ============================
                ## Keep record of state vector every 10 time steps
                state2 = state[1:, np.newaxis]
                
                if state_history is None:
                    state_history = state2 
                else:
                    if t_step%10==0: # Append state_history columns with current state2 vector 
                        state_history = np.append(state_history, state2, axis=1)
                        env.append_csv(agent,state,k_ep,k_run,sensor_data)

                if done_rollout==True:

                    env.step('stop')
                    reward[k_run] = agent.calculate_reward(state_history,h_ceiling)
                    print("Reward = %.3f" %(reward[k_run]))
                    print("!------------------------End Run------------------------! \n")
                    break

                t_step += 1
            
            ## =======  RUN COMPLETED  ======= ##

            ## CAP CSV WITH FINAL VALUES
            env.append_csv(agent,state,k_ep,k_run,sensor_data)
            env.IC_csv(agent,state,k_ep,k_run,policy,v_d,omega_d,reward[k_run,0],error_str)
            env.append_csv_blank()

            # ## UPDATE GLOBAL VARIABLES
            # K_EP.value = k_ep
            # K_RUN.value = k_run
            # REWARD.value = reward[k_run]
                       
            
            if repeat_run == True:
                env.close_sim()
                time.sleep(1)
                env.launch_sim()
                # return to previous run to catch potential missed glitches in gazebo (they are usually caught in the next run)
                if k_run > 0:
                    k_run -= 1 # Repeat previous run (not current)  
            else:
                k_run += 1 # Move on to next run
            
        ## =======  EPISODE COMPLETED  ======= ##
        if not any( np.isnan(reward) ):
            print("Episode # %d training, average reward %.3f" %(k_ep, np.mean(reward)))
            agent.train(theta_rl,reward,epsilon_rl)
            
        
    ## =======  MAX TRIALS COMPLETED  ======= ##







if __name__ == '__main__':
    # STATE = Array('d',14) # Global state array for Multiprocessing
    # REWARD = Value('d',0) 
    # REWARD_AVG = Value('d',0)
    # K_RUN = Value('i',0)
    # K_EP = Value('i',0)
    # N_ROLLOUTS = Value('i',0)

    # # START PLOTTING PROCESS
    # p1 = Process(target=runGraph,args=(STATE,K_EP,K_RUN,REWARD,REWARD_AVG,N_ROLLOUTS))
    # p1.start()

    ## START MAIN SCRIPT
    main()

