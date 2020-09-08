#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import time,copy, os

from crazyflie_env import CrazyflieEnv




## Initialize the environment
env = CrazyflieEnv(port_self=18050, port_remote=18060)
print("Environment done")


## Parameters
mu = np.array([[3.0], [-5.0]])
vz_ini = 3.0 #+ np.random.rand()
vx_ini = 0.0 #np.random.rand()
h_ceiling = 1.5

print("=============================================")
print("Parameters")
print("=============================================")

print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
print("Vz_ini: %.3f \t Vx_ini: %.3f" %(vz_ini, vx_ini))
print("RREV=%.3f, \t theta2=%.3f" %(mu[0], mu[1]))
print()


done = False
# reward = np.zeros(shape=(2*agent.n_rollout_,1))
# reward[:] = np.nan  # initialize reward to be NaN array, size n_rollout x 1


state = env.reset()

done_rollout = False
start_time_rollout = env.getTime()
start_time_pitch = None
pitch_triggered = False
track_state = None
        
#         env.logDataFlag = True

# ============================
##          Rollout 
# ============================
action = {'type':'vel', 'x':vx_ini, 'y':0.0, 'z':vz_ini, 'additional':0.0}
env.step(action=action)

RREV_trigger = mu[0]

print()
k_step = 0
while True:
    time.sleep(5e-4) # Time step size
    k_step = k_step + 1 # Time step

    ## Define current state
    state = env.state_current_
    
    position = state[1:4]
    orientation_q = state[4:8]
    vel = state[8:11]
    vz, vx = vel[2], vel[0]
    omega = state[11:14]

    d = h_ceiling - position[2]
    RREV, omega_y = vz/d, vx/d

    qw = orientation_q[0]
    qx = orientation_q[1]
    qy = orientation_q[2]
    qz = orientation_q[3]
    # theta = np.arcsin( -2*(qx*qz-qw*qy) ) # obtained from matlab "edit quat2eul.m"

    ## Enable sticky feet and rotation
    if (RREV > RREV_trigger) and (pitch_triggered == False):
        start_time_pitch = env.getTime()
        env.enableSticky(1)
        q_d = mu[1] * RREV

        print('------------- pitch starts -------------')
        print( 'vz=%.3f, vx=%.3f, RREV=%.3f, qd=%.3f' %(vz, vx, RREV, q_d) )   
        print("Pitch Time: %.3f" %start_time_pitch)
        
        env.delay_env_time(t_start=start_time_pitch,t_delay=30) # Artificial delay to mimic communication lag [ms]
        action = {'type':'rate', 'x':0.0, 'y':q_d, 'z':0.0, 'additional':0.0}
        env.step(action) # Start rotation and mark rotation triggered
        pitch_triggered = True


    
    ## If nan is found in state vector repeat sim run
    if any(np.isnan(state)): # gazebo sim becomes unstable, relaunch simulation
        print("NAN found in state vector")
        env.logDataFlag = False
        # env.close_sim()
        # env.launch_sim()
        break
   


    ## Keep record of state vector every 10 time steps
    state2 = state[1:,np.newaxis]
    if track_state is None:
        track_state = state2 # replace w/ state_history variable for track_state
    else:
        if k_step%10==0:
            track_state = np.append(track_state, state2, axis=1)

    if done_rollout:
        env.logDataFlag = False
        # reward[k_run] = agent.calculate_reward(_state=track_state, _h_ceiling=h_ceiling)
        break

    # env.add_record_xls(file_log=file_log, sheet=sheet, file_name=file_name,
    #     k_ep=k_ep, start_time1=start_time11, start_time2=start_time12,
    #     vz_ini=vz_ini, vx_ini=vx_ini, state=state, action=action[0],
    #     reward=reward, info=info, theta=theta)

