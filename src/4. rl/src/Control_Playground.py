#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import csv
import math

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
a_hist = []
t_hist = []

fig = plt.figure()
plt.grid()
plt.xlabel("Time (s)")
plt.ylabel("Pitch Angle (deg)")
plt.ylim([-10,10])



state = env.reset()

t_step =0

start_time_rolloout = env.getTime()
done = False

filename = "src/4. rl/src/log/km_test.csv"
with open(filename,mode='w') as csvfile:
    writer = csv.writer(csvfile,delimiter = ',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(['Time [s]','Omega_x','Omega_y','Omega_z'])



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


    qw,qx,qy,qz = orientation_q
    # R = Rotation.from_quat([qx,qy,qz,qw])
    # yaw,roll,pitch = R.as_euler('zxy',degrees=False)
    # print(R.as_euler('zxy',degrees=False))

    # a_hist.append(pitch*180/(math.pi))
    a_hist.append(omega[2])
    t_hist.append(t)


    with open(filename,mode='a') as csvfile:
        writer = csv.writer(csvfile,delimiter = ',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow([t,omega[0],omega[1],omega[2]])
        



    # ============================
    ##      Control Profiles
    # ============================


    if (1.0 <= t <= 1.1): 
        action = {'type':'pos', 'x':0.0, 'y':0.0, 'z':0.5, 'ctrl_flag':1}
        env.step(action)


    if (3.0 <= t):
        pass
        # turn on attitude control and pause sim
        # action = {'type':'att', 'x':0.0, 'y':0.0, 'z':0.0, 'ctrl_flag':1} 
        # env.step(action) # Set 5 deg pitch
    #     # os.system("""rosservice call /gazebo/set_model_state '{model_state: { model_name: crazyflie_landing_gears, pose: { position: { x: 0, y: 0 ,z: 0.35 }, orientation: {x: 0, y: 0.0436194, z: 0, w: 0.9990482 } }, twist: { linear: {x: 0 , y: 0 ,z: 0 } ,angular: { x: 0 , y: 0, z: 0 } } , reference_frame: world } }'""")
        # os.system("rosservice call gazebo/pause_physics")



    if (t >= 6.0):
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


plt.plot(t_hist,a_hist)
plt.show()

        
            



        







