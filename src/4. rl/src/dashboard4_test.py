#!/usr/bin/env python3

import numpy as np
import time,os,getpass
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

from multiprocessing import Process, Value, Array
from scipy.spatial.transform import Rotation




from crazyflie_env import CrazyflieEnv

def runGraph():
    # Parameters
    buffer = list(range(0, 200)) # Number of points to display
    buf_len = len(buffer)

    ## CREATE FIGURE FOR PLOTTING
    fig_0 = plt.figure(num = 0, figsize = (12, 8))
    ax_pos = plt.subplot2grid((2,2),(0,0))
    ax_vel = plt.subplot2grid((2,2),(1,0))
    ax_att = plt.subplot2grid((2,2),(0,1))
    ax_omega = plt.subplot2grid((2,2),(1,1))


    ## SET PLOT TITLES
    ax_pos.set_title('Position [m]')
    ax_vel.set_title('Velocity [m/s]')
    ax_att.set_title('Attitude [deg]')
    ax_omega.set_title('Ang. Rate [rad/s]')

    ## SET LABEL NAMES
    ax_pos.set_ylabel("Pos. x,y,z [m]")
    ax_vel.set_ylabel("Vel. x,y,z [m/s]")
    ax_att.set_ylabel("Att. x,y,z [deg]")
    ax_omega.set_ylabel("Ang. x,y,z [rad/s]")

    

    ## TURN ON GRIDS
    ax_pos.grid(True)
    ax_vel.grid(True)
    ax_att.grid(True)
    ax_omega.grid(True)

    ## SET Y-LIMITS
    ax_pos.set_ylim([-4,4])
    ax_vel.set_ylim([-4,4])
    ax_omega.set_ylim([-20,20])

    
    pos_x = [0]*buf_len
    pos_y = [0]*buf_len
    pos_z = [0]*buf_len
    
    vel_x = [0]*buf_len
    vel_y = [0]*buf_len
    vel_z = [0]*buf_len

    omega_x = [0]*buf_len
    omega_y = [0]*buf_len
    omega_z = [0]*buf_len


    

    # Create a blank line. We will update the line in animate
    line_px, = ax_pos.plot(buffer, pos_x,'b-',label="Pos x")
    line_py, = ax_pos.plot(buffer, pos_y,'g-',label="Pos y")
    line_pz, = ax_pos.plot(buffer, pos_z,'r-',label="Pos z")

    line_vx, = ax_vel.plot(buffer, vel_x,'b-',label="Vel x")
    line_vy, = ax_vel.plot(buffer, vel_y,'g-',label="Vel y")
    line_vz, = ax_vel.plot(buffer, vel_z,'r-',label="Vel z")

    line_wx, = ax_omega.plot(buffer, omega_x,'b-',label="omega x")
    line_wy, = ax_omega.plot(buffer, omega_y,'g-',label="omega y")
    line_wz, = ax_omega.plot(buffer, omega_z,'r-',label="omega z")

    ax_pos.legend([line_px,line_py,line_pz],[line_px.get_label(),line_py.get_label(),line_pz.get_label()])
    ax_vel.legend([line_vx,line_vy,line_vz],[line_vx.get_label(),line_vy.get_label(),line_vz.get_label()])
    ax_omega.legend([line_wx,line_wy,line_wz],[line_wx.get_label(),line_wy.get_label(),line_wz.get_label()])



    # This function is called periodically from FuncAnimation
    def animate(i,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,omega_x,omega_y,omega_z):

        # States from shared Multiprocessing array

        x_x,x_y,x_z = state_mp[1:4]
        qw,qx,qy,qz = state_mp[4:8]
        vx,vy,vz = state_mp[8:11]
        wx,wy,wz = state_mp[11:14]

        # R = Rotation.from_quat([qx,qy,qz,qw]) 


        ## POSITION LINES
        pos_x.append(x_x)  # Add y to list
        pos_x = pos_x[-buf_len:] # Limit y list to set number of items
        line_px.set_ydata(pos_x) # Update line with new Y values

        pos_y.append(x_y)  
        pos_y = pos_y[-buf_len:] 
        line_py.set_ydata(pos_y) 

        pos_z.append(x_z)  
        pos_z = pos_z[-buf_len:] 
        line_pz.set_ydata(pos_z)


        ## VELOCITY LINES
        vel_x.append(vx)  
        vel_x = vel_x[-buf_len:] 
        line_vx.set_ydata(vel_x) 

        vel_y.append(vy)  
        vel_y = vel_y[-buf_len:] 
        line_vy.set_ydata(vel_y) 

        vel_z.append(vz)  
        vel_z = vel_z[-buf_len:] 
        line_vz.set_ydata(vel_z) 


        ## ANG. RATE LINES
        omega_x.append(wx)
        omega_x = omega_x[-buf_len:]
        line_wx.set_ydata(omega_x)

        omega_y.append(wy)
        omega_y = omega_y[-buf_len:]
        line_wy.set_ydata(omega_y)

        omega_z.append(wz)
        omega_z = omega_z[-buf_len:]
        line_wz.set_ydata(omega_z)

        # ax_omega.set_ylim([-max(omega_z)-1,max(omega_z)+1])
        print(max(omega_y))


        line_wy.axes.set_ylim(-max(omega_y)*1.5-1,max(omega_y)*1.5+1)




        return line_px,line_py,line_pz,line_vx,line_vy,line_vz,line_wx,line_wy,line_wz


    # Set up plot to call animate() function periodically

    ani = animation.FuncAnimation(fig_0,
        animate,
        fargs=(pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,omega_x,omega_y,omega_z),
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
        # print(state[4:8])




if __name__ == '__main__':
    state_mp = Array('d',14)
    p1 = Process(target=runGraph)
    p1.start()
    Main()
    







            


            
            








        
        
