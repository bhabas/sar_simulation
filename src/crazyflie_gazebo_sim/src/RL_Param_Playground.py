#!/usr/bin/env python3

import numpy as np
import time,os,getpass
from scipy.spatial.transform import Rotation
import threading


from crazyflie_env import CrazyflieEnv
from rl_syspepg import ES,rlsysPEPGAgent_reactive

# from utility.dashboard import runGraph

os.system("clear")



# ============================
##     Sim Initialization 
# ============================



## INIT GAZEBO ENVIRONMENT
env = CrazyflieEnv()
# env.launch_dashboard()
print("Environment done")



## INIT USER AND DATA RECORDING
username = getpass.getuser()
start_time = time.strftime('_%Y-%m-%d_%H:%M:%S', time.localtime(time.time()))
file_name = '/home/'+username+'/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/log/' + username + start_time + '.csv'
env.create_csv(file_name,record = False)


## SIM PARAMETERS
ep_start = 0 # Default episode start position
h_ceiling = 1.5 # [m]

ES = ES(gamma=0.95,n_rollout=3)
agent = rlsysPEPGAgent_reactive(np.asarray(0),np.asarray(0),np.asarray(0),np.asarray(0))
reset_vals = True




# ============================
##          Episode 
# ============================
for k_ep in range(ep_start,1000):

    np.set_printoptions(precision=2, suppress=True)
    done = False

    omega_d = [0,0,0] # Junk declaration to cleanup warning or possible error
    reward = np.zeros(shape=(2*ES.n_rollout,1))

    

    print("=============================================")
    print("STARTING Episode # %d" %k_ep)
    print("=============================================")

    # ============================
    ##          Run 
    # ============================
    k_run = 0 # Reset run counter each episode
    while k_run < 2*ES.n_rollout:

        
        ## RESET TO INITIAL STATE
        state = env.reset_pos() # Reset Gazebo pos
        env.step('home',ctrl_flag=1) # Reset control vals and functionality to default vals
        

        

        if reset_vals == True:
    
            while True:
                try:
                    ## Mu input:
                    mu_str = input("Input mu values: ")
                    num = list(map(float, mu_str.split()))
                    mu = np.asarray(num)

                    if len(mu) != 2:
                        raise Exception()
                    break
                except:
                    print("Error: Enter mu_1 and mu_2")

            while True:
                try:
                    
                    v_str = input("Input V_ini (vx, vy, vz): ")
                    num = list(map(float, v_str.split()))
                    v_d = np.asarray(num)
                    vx_d,vy_d,vz_d = v_d
                    if len(v_d) != 3:
                        raise Exception()
                    break
                except:
                    print("Error: Enter vz,vx,vy")
            


        ## INITIALIZE RUN PARAMETERS
        RREV_trigger = mu[0] # FoV expansion velocity [rad/s]
        G1 = mu[1]
        G2 = 0

        policy = np.array([RREV_trigger,G1,G2])
        policy = policy[:,np.newaxis] # reshaping for data logging [ change [3,] -> [3,1] ]

        ## RREV: 5.5, G1: 7.6, Vz_d: 3.0 works very well for landing




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
            state = np.array(env.state_current)
            
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

                print('======= Flip Starts =======')
                print('vx=%.3f, vy=%.3f, vz=%.3f' %(vx,vy,vz))
                print('RREV=%.3f, OF_y=%.3f, OF_x=%.3f' %(RREV, OF_y, OF_x))   
                print('Pitch Time: %.3f, Omega_yd=%.3f' %(start_time_pitch,omega_yd))

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

                w_y_hist = state_history[11,:]
                max_omega_y = min(np.max(abs(w_y_hist)),30)
                reward[k_run] = agent.calculate_reward(state_history,h_ceiling)

                print("Max Omega_y= %.3f" %(max_omega_y))
                print("Reward = %.3f" %(reward[k_run]))
                print("!------------------------End Run------------------------! \n")
                break

            t_step += 1
        
        ## =======  RUN COMPLETED  ======= ##

        ## CAP CSV WITH FINAL VALUES
        env.append_csv(agent,state,k_ep,k_run,sensor_data)
        env.IC_csv(agent,state,k_ep,k_run,policy,v_d,omega_d,reward[k_run,0],error_str)
        env.append_csv_blank()

        
        
                    
        
        if repeat_run == True:
            env.close_sim()
            time.sleep(1)
            env.launch_sim()
        
        else:
            ## UPDATE PUBLISHED REWARD VARIABLES
            env.n_rollouts = agent.n_rollout*2
            env.k_ep = k_ep
            env.k_run = k_run
            env.reward = reward[k_run,0]
            env.reward_avg = np.mean(reward[:k_run+1,0])
            env.rewardPub()

            str = input("Input: \n(1): To play again \n(2): To repeat scenario \n(3): Game over :( \n")
            if str == '1':
                k_run += 1
                reset_vals = True

            elif str == '2':
                k_run += 1
                reset_vals = False
            else: 
                break

        
        


        
    ## =======  EPISODE COMPLETED  ======= ##
    # if not any( np.isnan(reward) ):
    print("Episode # %d training, average reward %.3f" %(k_ep, np.mean(reward)))
    
        
    
## =======  MAX TRIALS COMPLETED  ======= ##


