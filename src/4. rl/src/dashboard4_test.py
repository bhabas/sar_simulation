#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import matplotlib.pyplot as plt
import os


from crazyflie_env import CrazyflieEnv


env = CrazyflieEnv()
print("Environment done")


np.set_printoptions(precision=2, suppress=True)
env.step('home',ctrl_flag=1) # Reset control vals and functionality
state = env.reset_pos() # Reset Gazebo pos


while True:
        

    ## DEFINE CURRENT STATE [Can we thread this to get states even when above]
    state = env.state_current
    print(state)







            


            
            








        
        
