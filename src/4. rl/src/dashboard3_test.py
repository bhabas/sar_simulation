#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import matplotlib.pyplot as plt
from math import sin,cos,pi,sqrt,atan,atan2
import os
from scipy.spatial.transform import Rotation
from numpy.core.fromnumeric import repeat

from crazyflie_env import CrazyflieEnv
from multiprocessing import Process,Value


def main():
    ## Initialize the environment
    env = CrazyflieEnv()
    state = env.reset_pos()

    while True:

        time.sleep(5e-4) # Time step size

        ## Define current state
        state = env.state_current
        print(state)
        # total.value += 1


def print_val():
    while True:
        time.sleep(0.01)
        # print(total.value)


if __name__ == '__main__':
    total = Value('d',0)

    add_process = Process(target=main,args=())
    # sub_process = Process(target=print_val,args=())

    add_process.start()
    # sub_process.start()

    add_process.join()
    # sub_process.join()
