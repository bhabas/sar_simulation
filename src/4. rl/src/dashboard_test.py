import matplotlib.pyplot as plt
import matplotlib.animation as animation
from multiprocessing import Process
import numpy as np
import time


import time,os,getpass
import os
from scipy.spatial.transform import Rotation
from crazyflie_env import CrazyflieEnv

# https://stackoverflow.com/questions/51949185/non-blocking-matplotlib-animation

def runGraph():
    # Parameters
    print('show')
    x_len = 200         # Number of points to display
    y_range = [10, 40]  # Range of possible Y values to display

    # Create figure for plotting
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    xs = list(range(0, 200))
    ys = [0] * x_len
    ax.set_ylim(y_range)

    # Create a blank line. We will update the line in animate
    line, = ax.plot(xs, ys)

    # Add labels
    plt.title('TMP102 Temperature over Time')
    plt.xlabel('Samples')
    plt.ylabel('Temperature (deg C)')

    # This function is called periodically from FuncAnimation
    def animate(i, ys):

        # Read temperature (Celsius) from TMP102
        temp_c = np.random.random(1)*40

        # Add y to list
        ys.append(temp_c)

        # Limit y list to set number of items
        ys = ys[-x_len:]

        # Update line with new Y values
        line.set_ydata(ys)

        return line,


    # Set up plot to call animate() function periodically

    ani = animation.FuncAnimation(fig,
        animate,
        fargs=(ys,),
        interval=50,
        blit=True)
    plt.show()


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