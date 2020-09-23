#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import matplotlib.pyplot as plt
from math import sin,cos,pi
import os
from scipy.spatial.transform import Rotation
from numpy.core.fromnumeric import repeat

from crazyflie_env import CrazyflieEnv
from rl_syspepg import rlsysPEPGAgent_reactive

os.system("clear")

# ============================
##     Sim Initialization 
# ============================


## Initialize the environment
env = CrazyflieEnv(port_self=18050, port_remote=18060)
print("Environment done")



## Initialize the user and data recording
username = getpass.getuser()
start_time = time.strftime('_%Y-%m-%d_%H:%M:%S', time.localtime(time.time()))
file_name = '/home/'+username+'/catkin_ws/src/crazyflie_simulation/src/4. rl/src/log/' + username + start_time + '_testenv.csv'
env.create_csv(file_name,record = True)

## Initial variables
h_ceiling = 1.5
k_run = 0


while True:

    ## Mu input:
    mu_str = input("Input mu values: ")
    num = list(map(float, mu_str.split()))
    mu = np.asarray(num)

    ## Placeholders need to be arrays for formatting reasons
    agent = rlsysPEPGAgent_reactive(np.asarray(0),np.asarray(0),mu,np.asarray(0))

    ## v_ini input:
    v_str = input("Input V_ini (vz,vx,vy): ")
    num = list(map(float, v_str.split()))
    v_ini = np.asarray(num)

    vz_ini,vx_ini,vy_ini = v_ini

    # vz_ini = 3.0
    # vx_ini = 0.0 
    # vy_ini = 0.0
    


    print("=============================================")
    print("              Trial Run: %d" %k_run)
    print("=============================================")


    print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
    print("Vz_ini: %.3f \t Vx_ini: %.3f \t  Vy_ini:%.3f" %(vz_ini, vx_ini, vy_ini))
    print("RREV=%.3f, \t Gain=%.3f" %(mu[0], mu[1]))
    print()



    state = env.reset()
    env.IC_csv(agent,np.around(state,decimals=3),'sim',k_run,v_ini = v_ini)

    ## If spawn position is off then reset position again
    x_pos,y_pos = state[2],state[3]
    print("Spawn Pos: x=%.3f \t y=%.3f" %(x_pos,y_pos))
    if abs(x_pos) > 0.1 or abs(y_pos) > 0.1:
        state = env.reset()
    

    action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
    env.step(action)


    done = False
    done_rollout = False
    flip_flag = False

    start_time_rollout = env.getTime()
    start_time_pitch = None
    pitch_triggered = False

    state_history = None

    # ============================
    ##          Rollout 
    # ============================
    action = {'type':'vel', 'x':vx_ini, 'y':vy_ini, 'z':vz_ini, 'additional':0.0}
    env.step(action=action)
    RREV_trigger = mu[0]

    error_str = ''
    reward = 0
    t_step=0
    while True:
                
        time.sleep(5e-4) # Time step size
        t_step = t_step + 1 # Time step

        ## Define current state
        state = env.state_current

        position = state[1:4]
        orientation_q = state[4:8]
        vel = state[8:11]
        vx,vy,vz = vel
        omega = state[11:14]

        d = h_ceiling - position[2]
        RREV, omega_y ,omega_x = vz/d, (vx**2)/d, vy/d

        qw = orientation_q[0]
        qx = orientation_q[1]
        qy = orientation_q[2]
        qz = orientation_q[3]

        R = Rotation.from_quat([qx,qy,qz,qw])
        R = R.as_matrix()
        angle = R[1,1]
        b3 = R[2,:]
        #print(angle, b3[0],b3[1],b3[2])



        ## First time CF flips past 90 deg turn off motors 
        if b3[2] < 0.0 and not flip_flag:
            #print(angle)
            action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
            env.step(action)
            print("Flipped!! Motors shut down")
            flip_flag = True
        


        #b3 = r.as_matrix()[:,2] # body z-axis
        #b3y =  np.dot(b3, np.array([0,0,1]))
        b3y=0
        r = [0,0,0]#r.as_euler('zyx', degrees=True)
        eul1,eul2,eul3 = r[0],r[1],r[2]
        #print(r)
        theta = np.arcsin( -2*(qx*qz-qw*qy) ) # obtained from matlab "edit quat2eul.m"


        # ============================
        ##    Pitch Criteria 
        # ============================
        if (RREV > RREV_trigger) and (pitch_triggered == False):
            start_time_pitch = env.getTime()
            env.enableSticky(1)

            # add term to adjust for tilt 
            qRREV = mu[1] * RREV 
            # qomega = theta_rl[2,k_run]*(omega[1])
            q_d = qRREV # + qomega #omega_x#*(1-b3y)#sin(r[1]*3.14159/180)
            # torque on x axis to adjust for vy
            r_d = 0.0 #theta_rl[3,k_run] * omega_y

            print('----- pitch starts -----')
            print('vz=%.3f, vx=%.3f, vy=%.3f' %(vz,vx,vy))
            print('r[0]=%.3f, r[1]=%.3f, r[2]=%.3f, b3y=%.3f' %(r[0],r[1],r[2],b3y))
            print('RREV=%.3f, omega_y=%.3f, omega_x=%.3f, qd=%.3f' %( RREV, omega_y, omega_x,q_d) ) 
            print()  
            print("Pitch Time: %.3f" %(start_time_pitch))
            #print('wy = %.3f , qomega = %.3f , qRREV = %.3f' %(omega[1],qomega,qRREV))
            
            env.delay_env_time(t_start=start_time_pitch,t_delay=30) # Artificial delay to mimic communication lag [ms]
            action = {'type':'rate', 'x':r_d, 'y':q_d, 'z':0.0, 'additional':0.0}
            env.step(action) # Start rotation and mark rotation triggered
            pitch_triggered = True


        # ============================
        ##    Termination Criteria 
        # ============================

        ## If time since triggered pitch exceeds [0.7s]   
        if pitch_triggered and ((env.getTime()-start_time_pitch) > 0.7):
            # I don't like this formatting, feel free to improve on
            error_1 = "Rollout Completed: Pitch Triggered  "
            error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_pitch,(env.getTime()-start_time_pitch))
            print(error_1 + "\n" + error_2)

            error_str = error_1 + error_2
            done_rollout = True

        ## If time since rollout start exceeds [1.5s]
        if (env.getTime() - start_time_rollout) > 1.5:
            error_1 = "Rollout Completed: Time Exceeded   "
            error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_rollout,(env.getTime()-start_time_rollout))
            print(error_1 + "\n" + error_2)

            error_str = error_1 + error_2
            done_rollout = True

        ## If position exceeds ceiling bounds mark error
        if (np.abs(position[0]) > 3.0) or (np.abs(position[1]) > 3.0):
            error_str = "Reset improperly/Position outside bounding box"
            print(error_str)
            break
        

        # ============================
        ##          Errors  
        # ============================
        ## If nan is found in state vector repeat sim run
        if any(np.isnan(state)) or env.timeout: # gazebo sim becomes unstable, relaunch simulation
            print("NAN found in state vector")
            print(env.timeout)
            print(state)
            time.sleep(1)

            env.close_sim()
            env.launch_sim()

            error_str = "Error: NAN found in state vector"
            repeat_run = True
            break



        
        ## Keep record of state vector every 10 time steps
        temp = state[1:].reshape(-1,1) # Remove time from state vector and reshape to [13,1]
        if state_history is None:
            state_history = temp # Instantiate state_history variable
        else:
            if t_step%10==0:
                state_history = np.append(state_history, temp, axis=1)
                env.append_csv(agent,np.around(state,decimals=3),'sim',k_run)



        if done_rollout:
            action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
            env.step(action)
            reward = agent.calculate_reward(state_history,h_ceiling)
            print(reward)
            

            print("!------------------------End Run------------------------! \n")
            break
        


    env.append_csv(agent,np.around(state,decimals=3),'sim',k_run,reward,error=error_str)
    env.append_csv_blank()

    run = bool(input("Enter to Run Again:"))
    if run == True: # If input other than 'Enter' break to end of script
        break
    else:
        k_run += 1



