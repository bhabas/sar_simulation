#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

from multiprocessing import Process, Value, Array




from crazyflie_env import CrazyflieEnv

def runGraph():
    # Parameters
    print('show')
    x_len = 200         # Number of points to display
    y_range = [0,4]  # Range of possible Y values to display

    # Create figure for plotting
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    xs = list(range(0, 200))
    ys = [0] * x_len
    ax.set_ylim(y_range)

    # Create a blank line. We will update the line in animate
    line, = ax.plot(xs, ys)

    # Add labels
    plt.xlabel('Samples')
    plt.ylabel('Z-Position [m]')

    # This function is called periodically from FuncAnimation
    def animate(i, ys):

        # Read temperature (Celsius) from TMP102
        pos_z = state_mp[3]

        # Add y to list
        ys.append(pos_z)

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


def print_val():
    while True:
        time.sleep(0.01)
        print(state_mp[1:4])


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




if __name__ == '__main__':
    state_mp = Array('d',14)
    p1 = Process(target=runGraph)
    p1.start()
    Main()
    







            


            
            








        
        
