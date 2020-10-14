#!/usr/bin/env python3

import numpy as np
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



while True:

    state = env.reset()
    t_step =0


  
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
        
            



        







