#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import matplotlib.pyplot as plt
import os

from multiprocessing import Process, Value


from crazyflie_env import CrazyflieEnv

def print_val():
    while True:
        time.sleep(0.01)
        print(total.value)


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
        total.value = state[0]
        # print(state[0])



if __name__ == '__main__':
    total = Value('d',0)
    p1 = Process(target=print_val)
    p1.start()
    Main()
    







            


            
            








        
        
