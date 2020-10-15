#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

import time,os,getpass
import os
from scipy.spatial.transform import Rotation


from crazyflie_env import CrazyflieEnv


os.system("clear")
username = getpass.getuser()

# ============================
##     Sim Initialization 
# ============================


## Initialize the environment
env = CrazyflieEnv(port_self=18050, port_remote=18060,username =username)
print("Environment done")
pos_hist = []
t_hist = []

fig = plt.figure()
plt.grid()
plt.xlabel("Time (s)")
plt.ylim([0,1.5])



state = env.reset()
t_step =0

start_time_rolloout = env.getTime()
done = False



while True:



                
            
    time.sleep(5e-4) # Time step size
    t_step = t_step + 1 # Time step

    ## Define current state
    state = env.state_current

    pos = state[1:4]
    orientation_q = state[4:8]
    vel = state[8:11]
    vx,vy,vz = vel
    omega = state[11:14]
    t = state[0]


    if (1.0 <= t <= 1.5): 
        action = {'type':'pos', 'x':0.0, 'y':0.0, 'z':0.75, 'ctrl_flag':1}
        env.step(action)

    if (3.5 <= t <= 3.55):
        action = {'type':'pos', 'x':0.0, 'y':0.0, 'z':1.5, 'ctrl_flag':1}
        env.step(action)
        action = {'type':'vel', 'x':0.0, 'y':0.0, 'z':0.0, 'ctrl_flag':0}
        env.step(action)

    qw,qx,qy,qz = orientation_q
    R = Rotation.from_quat([qx,qy,qz,qw])
    yaw,roll,pitch = R.as_euler('zxy',degrees=False)
    # print(R.as_euler('zxy',degrees=False))

    pos_hist.append(pos[2])
    t_hist.append(t)

    # print(env.getTime() - start_time_rolloout)
    if (env.getTime() - start_time_rolloout) > (5):
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


plt.plot(t_hist,pos_hist)
plt.show()

        
            



        







