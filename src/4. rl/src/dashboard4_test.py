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
    buf_len = 200         # Number of points to display
    y_range = [0,4]  # Range of possible Y values to display

    # Create figure for plotting
    fig_0 = plt.figure(num = 0, figsize = (12, 8))
    ax1 = plt.subplot2grid((2,2),(0,0))
    ax2 = plt.subplot2grid((2,2),(1,0))



    buffer = list(range(0, 200))
    pos_x = [0]*buf_len
    pos_y = [0]*buf_len
    pos_z = [0]*buf_len
    

    vel_x = [0] * buf_len
    vel_y = [0] * buf_len
    vel_z = [0] * buf_len


    ax1.set_ylim(y_range)
    ax2.set_ylim([-4,4])

    # Create a blank line. We will update the line in animate
    line_px, = ax1.plot(buffer, pos_x)
    line_py, = ax1.plot(buffer, pos_y)
    line_pz, = ax1.plot(buffer, pos_z)

    line_vx, = ax2.plot(buffer, vel_x)
    line_vy, = ax2.plot(buffer, vel_y)
    line_vz, = ax2.plot(buffer, vel_z)



    # This function is called periodically from FuncAnimation
    def animate(i,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z):

        # States from shared Multiprocessing array

        x_x,x_y,x_z = state_mp[1:4]
        qw,qx,qy,qz = state_mp[4:8]
        vx,vy,vz = state_mp[8:11]
        omega_x,omega_y,omega_z = state_mp[11:14]


        pos_x.append(x_x)  # Add y to list
        pos_x = pos_x[-buf_len:] # Limit y list to set number of items
        line_px.set_ydata(pos_x) # Update line with new Y values

        pos_y.append(x_y)  
        pos_y = pos_y[-buf_len:] 
        line_py.set_ydata(pos_y) 

        pos_z.append(x_z)  
        pos_z = pos_z[-buf_len:] 
        line_pz.set_ydata(pos_z)


        vel_x.append(vx)  
        vel_x = vel_x[-buf_len:] 
        line_vx.set_ydata(vel_x) 

        vel_y.append(vy)  
        vel_y = vel_y[-buf_len:] 
        line_vy.set_ydata(vel_y) 

        vel_z.append(vz)  
        vel_z = vel_z[-buf_len:] 
        line_vz.set_ydata(vel_z) 

        return line_px,line_py,line_pz,line_vx,line_vy,line_vz


    # Set up plot to call animate() function periodically

    ani = animation.FuncAnimation(fig_0,
        animate,
        fargs=(pos_x,pos_y,pos_z,vel_x,vel_y,vel_z),
        interval=50,
        blit=True)
    plt.show()



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
    







            


            
            








        
        
