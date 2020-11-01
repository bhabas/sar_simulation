#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

from multiprocessing import Process, Value, Array
from scipy.spatial.transform import Rotation

from dashboard import runGraph
from crazyflie_env import CrazyflieEnv

os.system("clear")



def Main():
    env = CrazyflieEnv()
    print("Environment done")

    np.set_printoptions(precision=2, suppress=True)
    env.step('home',ctrl_flag=1) # Reset control vals and functionality
    state = env.reset_pos() # Reset Gazebo pos





    while True:

        time.sleep(5e-4) # Time step size

        ## DEFINE CURRENT STATE [Can we thread this to get states even when above]
        state = env.state_current
        state_mp[:] = state.tolist()
        # print(state[4:8])




if __name__ == '__main__':
    state_mp = Array('d',14)
    p1 = Process(target=runGraph,args=(state_mp,))
    p1.start()
    Main()
    







            


            
            








        
        
