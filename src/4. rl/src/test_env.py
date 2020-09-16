#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import time, copy, os
import pos_and_vel as pv

from crazyflie_env import CrazyflieEnv


os.system("clear")

# ------------------------ Initialize the environment ------------------------ #

env = CrazyflieEnv(port_self=18050, port_remote=18060)
print("Environment done")







run = True
while run == True:

# -------------------------------- Parameters -------------------------------- #

    vz_ini = 0.5 #+ np.random.rand()
    vx_ini = 0.0 #np.random.rand()
    h_ceiling = 1.5


    ## Mu input:
    s = input("Input mu:")
    num = list(map(float, s.split()))
    num = np.asarray(num)
    mu = num[:2]
    vel = num[2:]
    # mu = np.array([[3.0], [-5.0]])


    print("=============================================")
    print("Parameters")
    print("=============================================")

    print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
    print("Vz_ini: %.3f \t Vx_ini: %.3f" %(vz_ini, vx_ini))
    print("RREV=%.3f, \t Gain=%.3f" %(mu[0], mu[1]))
    print()

    done = False

    state = env.reset()

    rollout_complete = False
    start_time_rollout = env.getTime()
    start_time_pitch = None
    pitch_triggered = False
    state_history = None



    
# ---------------------------------- Rollout --------------------------------- #
    action = {'type':'vel', 'x':vx_ini, 'y':0.0, 'z':vel, 'additional':0.0}
    env.step(action=action)
    # pv.gazebo_IC(vz=vel[0])

    RREV_trigger = mu[0]

    t_step = 0
    while True:
        time.sleep(5e-4) # Time step size
        t_step = t_step + 1 # Time step

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

        ## If time since triggered pitch exceeds [0.7s]   
        if pitch_triggered and ((env.getTime()-start_time_pitch) > 0.7):
            print("Rollout Completed: Pitch Triggered")
            print("Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_pitch,(env.getTime()-start_time_pitch)))
            rollout_complete = True

        ## If time since rollout start exceeds [3s]
        if (env.getTime() - start_time_rollout) > 3:
            print("Rollout Completed: Time Exceeded")
            print("Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_rollout,(env.getTime()-start_time_rollout)))
            done_rollout = True

        ## If nan is found in state vector repeat sim run
        if any(np.isnan(state)): # gazebo sim becomes unstable, relaunch simulation
            print("NAN found in state vector")
            env.logDataFlag = False
            # env.close_sim()
            # env.launch_sim()
            break

        ## Keep record of state vector every 10 time steps
        state2 = state[1:,np.newaxis]
        if state_history is None:
            state_history = state2 # replace w/ state_history variable for track_state
        else:
            if t_step%10==0:
                state_history = np.append(state_history, state2, axis=1)

        if rollout_complete:
            env.logDataFlag = False
            # reward[k_run] = agent.calculate_reward(_state=track_state, _h_ceiling=h_ceiling)
            break
    
    run = bool(input("Run again?:"))


