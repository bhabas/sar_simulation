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


## Sim Parameters
h_ceiling = 1.5 # meters
extra_time = 0.2
k_run = 0
reset_vals = True

while True:

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

        ## Placeholders need to be arrays for formatting reasons
        agent = rlsysPEPGAgent_reactive(np.asarray(0),np.asarray(0),mu,np.asarray(0))

        while True:
            try:
                
                v_str = input("Input V_ini (vz,vx,vy): ")
                num = list(map(float, v_str.split()))
                v_ini = np.asarray(num)
                vz_ini,vx_ini,vy_ini = v_ini
                if len(v_ini) != 3:
                    raise Exception()
                break
            except:
                print("Error: Enter vz,vx,vy")
    


    print("=============================================")
    print("              Trial Run: %d" %k_run)
    print("=============================================")


    print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
    print("RREV=%.3f, \t Gain=%.3f" %(mu[0], mu[1]))
    print("Vz_ini: %.3f \t Vx_ini: %.3f \t  Vy_ini:%.3f" %(vz_ini, vx_ini, vy_ini))
    print('\n')



    state = env.reset()
    env.IC_csv(agent,state,'sim',k_run,v_ini = v_ini)

    ## If spawn position is off then reset position again
    x_pos,y_pos = state[2],state[3]
    print("Spawn Pos: x=%.3f \t y=%.3f" %(x_pos,y_pos))
    if abs(x_pos) > 0.1 or abs(y_pos) > 0.1:
        state = env.reset()
    

    action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
    env.step(action)

    t_step =0
    done_rollout = False
    start_time_rollout = env.getTime()
    start_time_pitch = None
    pitch_triggered = False
    flip_flag = False
    crash_flag = False

    state_history = None
    error_str = ''

    done = False
    repeat_run = False
    reward = 0
    
   

    # ============================
    ##          Rollout 
    # ============================
    action = {'type':'vel', 'x':vx_ini, 'y':vy_ini, 'z':vz_ini, 'additional':0.0}
    env.step(action=action)
    RREV_trigger = mu[0]

    


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
        RREV, OF_y, OF_x = vz/d, vx/d, vy/d # OF_x,y are optical flows of the ceiling assuming no body rotation
        sensor_data = [RREV, OF_y, OF_x] # simplifying for data recording

        qw = orientation_q[0]
        qx = orientation_q[1]
        qy = orientation_q[2]
        qz = orientation_q[3]

        R = Rotation.from_quat([qx,qy,qz,qw])
        R = R.as_matrix()
        angle = R[1,1]
        b3 = R[2,:]
        #print(angle, b3[0],b3[1],b3[2])


        # ============================
        ##    Motor Shutdown Criteria 
        # ============================
        if b3[2] < 0.0 and not flip_flag:
            # check if crazyflie flipped 
            action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
            env.step(action)
            # shut off motors for the rest of the run
            print("Flipped!! Shut motors")
            flip_flag = True

        if ((d < 0.05) and (not crash_flag) and (not flip_flag)): # might need to adjust this 
            action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
            env.step(action)
            print("Crashed!! Shut motors")
            crash_flag = True

        #b3 = r.as_matrix()[:,2] # body z-axis
        #b3y =  np.dot(b3, np.array([0,0,1]))
        b3y=0
        r = [0,0,0]#r.as_euler('zyx', degrees=True)
        eul1,eul2,eul3 = r[0],r[1],r[2]
        #print(r)
  

        # ============================
        ##    Pitch Criteria 
        # ============================
        if (RREV > RREV_trigger) and (pitch_triggered == False):
            start_time_pitch = env.getTime()
            env.enableSticky(1)

            # add term to adjust for tilt 
            qRREV = mu[1] * RREV 
            # qomega = theta_rl[2,k_run]*(omega[1])
            q_pitch = qRREV # + qomega #omega_x#*(1-b3y)#sin(r[1]*3.14159/180)
            # torque on x axis to adjust for vy
            q_roll = 0.0 #theta_rl[3,k_run] * omega_y

            print('----- pitch starts -----')
            print('vz=%.3f, vx=%.3f, vy=%.3f' %(vz,vx,vy))
            print('r[0]=%.3f, r[1]=%.3f, r[2]=%.3f, b3y=%.3f' %(r[0],r[1],r[2],b3y))
            print('RREV=%.3f, OF_y=%.3f, OF_x=%.3f, q_pitch=%.3f' %( RREV, OF_y, OF_x,q_pitch) ) 
            print("Pitch Time: %.3f" %(start_time_pitch))
            #print('wy = %.3f , qomega = %.3f , qRREV = %.3f' %(omega[1],qomega,qRREV))
            
            env.delay_env_time(t_start=start_time_pitch,t_delay=30) # Artificial delay to mimic communication lag [ms]
            action = {'type':'rate', 'x':q_roll, 'y':q_pitch, 'z':0.0, 'additional':0.0}
            env.step(action) # Start rotation and mark rotation triggered
            pitch_triggered = True


        # ============================
        ##    Termination Criteria 
        # ============================

        ## If time since triggered pitch exceeds [0.7s]   
        if pitch_triggered and ((env.getTime()-start_time_pitch) > 0.7 + extra_time):
            # I don't like this formatting, feel free to improve on
            error_1 = "Rollout Completed: Pitch Triggered  "
            error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_pitch,(env.getTime()-start_time_pitch))
            print(error_1 + "\n" + error_2)

            error_str = error_1 + error_2
            done_rollout = True

        ## If time since rollout start exceeds [1.5s]
        if (env.getTime() - start_time_rollout) > (1.5 + extra_time):
            error_1 = "Rollout Completed: Time Exceeded   "
            error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_rollout,(env.getTime()-start_time_rollout))
            print(error_1 + "\n" + error_2)

            error_str = error_1 + error_2
            done_rollout = True

        ## If position exceeds ceiling bounds mark error
        if (np.abs(position[0]) > 20) or (np.abs(position[1]) > 20):
            error_str = "Reset improperly/Position outside bounding box"
            print(error_str)
            break
        

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
        
        elif (np.abs(position[0]) > 5.0) or (np.abs(position[1]) > 5.0):
            # might just need to increase bounds (not necessarily an error)
            print("Reset improperly / Out of bounds !!!!!")
            error_str = "Reset improperly/Position outside bounding box"
            repeat_run = True
            break
            


        # ============================
        ##      Record Keeping  
        # ============================
        ## Keep record of state vector every 10 time steps
        temp = state[1:].reshape(-1,1) # Remove time from state vector and reshape to [13,1]
        if state_history is None:
            state_history = temp # Instantiate state_history variable
        else:
            if t_step%10==0:
                state_history = np.append(state_history, temp, axis=1)
                env.append_csv(agent,state,'sim',k_run,sensor_data)



        if done_rollout:
            action = {'type':'stop', 'x':0.0, 'y':0.0, 'z':0.0, 'additional':0.0}
            env.step(action)
            reward = agent.calculate_reward(state_history,h_ceiling)
            print(reward)
            

            print("!------------------------End Run------------------------! \n")
            break
        


    env.append_csv(agent,state,'sim',k_run,sensor_data,reward,error=error_str)
    env.append_csv_blank()


    if repeat_run == True:
        time.sleep(1)
        env.close_sim()
        env.launch_sim()

    str = input("Input: \n(1): To play again \n(2): To repeat scenario \n(3): Game over :( \n")
    if str == '1':
        k_run += 1
        reset_vals = True

    elif str == '2':
        reset_vals = False
    else: 
        break




