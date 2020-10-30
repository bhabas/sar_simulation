import matplotlib.pyplot as plt
import matplotlib.animation as animation
from multiprocessing import Process
import numpy as np



import time,os,getpass
from scipy.spatial.transform import Rotation
from crazyflie_env import CrazyflieEnv

# https://stackoverflow.com/questions/51949185/non-blocking-matplotlib-animation

def runGraph():
    pass
   


def MainProgram():

    # ============================
    ##     Sim Initialization 
    # ============================


    ## Initialize the environment
    env = CrazyflieEnv()
    state = env.reset_pos()

    while True:

        time.sleep(5e-4) # Time step size

        ## Define current state
        state = env.state_current

        t = state[0]
        pos = state[1:4]
        orientation_q = state[4:8]
        vel = state[8:11]
        vx,vy,vz = vel
        omega = state[11:14]
        qw,qx,qy,qz = orientation_q




if __name__ == '__main__':
    p = Process(target=runGraph)
    p.start()
    MainProgram()
    p.join()