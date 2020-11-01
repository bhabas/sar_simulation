#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

from multiprocessing import Process, Value, Array
from scipy.spatial.transform import Rotation



def runGraph(state_G,reward_G,k_run_G,k_ep_G):
    # Parameters
    buf_len = 600 # num datapoints in plot
    interval = 50 # Interval between plot frames [ms]
    sec_hist = buf_len*interval/1000 # Time shown on dashboard [s]


    buffer = list(range(0,buf_len)) # Number of points to display
    
    ## CREATE FIGURE FOR PLOTTING
    fig_0 = plt.figure(0, figsize = (12,6))
    
    ax1_pos = plt.subplot2grid((2,3),(0,0))
    ax1_vel = plt.subplot2grid((2,3),(0,1))
    ax1_omega = plt.subplot2grid((2,3),(0,2))
    ax1_att = plt.subplot2grid((2,3),(1,0))
    ax1_ms = plt.subplot2grid((2,3),(1,1),colspan=2,rowspan=1)

    axes1 = [ax1_pos,ax1_vel,ax1_att,ax1_omega,ax1_ms]


    ## SET PLOT TITLES
    ax1_pos.set_title('Position [m]')
    ax1_vel.set_title('Velocity [m/s]')
    ax1_att.set_title('Attitude [deg]')
    ax1_omega.set_title('Ang. Rate [rad/s]')
    ax1_ms.set_title('Motor speeds [rad/s]')

    ## SET LABEL NAMES
    ax1_pos.set_ylabel("Pos. x,y,z [m]")
    ax1_vel.set_ylabel("Vel. x,y,z [m/s]")
    ax1_att.set_ylabel("Att. x,y,z [deg]")
    ax1_omega.set_ylabel("Ang. x,y,z [rad/s]")
    ax1_ms.set_ylabel("MS [rad/s]")


    ## SET Y-LIMITS
    ax1_pos.set_ylim([-2,2])
    ax1_vel.set_ylim([-3,3])
    ax1_att.set_ylim([-200,200])
    ax1_omega.set_ylim([-30,30])
    ax1_ms.set_ylim([0,2750])

    

    
    pos_x = [0]*buf_len
    pos_y = [0]*buf_len
    pos_z = [0]*buf_len
    
    vel_x = [0]*buf_len
    vel_y = [0]*buf_len
    vel_z = [0]*buf_len

    att_x = [0]*buf_len
    att_y = [0]*buf_len
    att_z = [0]*buf_len

    omega_x = [0]*buf_len
    omega_y = [0]*buf_len
    omega_z = [0]*buf_len

    MS_1 = [0]*buf_len
    MS_2 = [0]*buf_len
    MS_3 = [0]*buf_len
    MS_4 = [0]*buf_len

    

    # Create a blank line. We will update the line in animate
    line_px, = ax1_pos.plot(buffer, pos_x,'b-',label="Pos. X")
    line_py, = ax1_pos.plot(buffer, pos_y,'g-',label="Pos. Y")
    line_pz, = ax1_pos.plot(buffer, pos_z,'r-',label="Pos. Z")

    line_vx, = ax1_vel.plot(buffer, vel_x,'b-',label="Vel. X")
    line_vy, = ax1_vel.plot(buffer, vel_y,'g-',label="Vel. Y")
    line_vz, = ax1_vel.plot(buffer, vel_z,'r-',label="Vel. Z")

    line_ax, = ax1_att.plot(buffer, att_x,'b-',label="Roll")
    line_ay, = ax1_att.plot(buffer, att_y,'g-',label="Pitch")
    line_az, = ax1_att.plot(buffer, att_z,'r-',label="Yaw")

    line_wx, = ax1_omega.plot(buffer, omega_x,'b-',label='$\omega_x$')
    line_wy, = ax1_omega.plot(buffer, omega_y,'g-',label='$\omega_y$')
    line_wz, = ax1_omega.plot(buffer, omega_z,'r-',label='$\omega_z$')

    line_ms1, = ax1_ms.plot(buffer, MS_1,color = 'steelblue',label="MS: 1")
    line_ms2, = ax1_ms.plot(buffer, MS_2,'b-',label="MS: 2")
    line_ms3, = ax1_ms.plot(buffer, MS_3,'g-',label="MS: 3")
    line_ms4, = ax1_ms.plot(buffer, MS_4,'r-',label="MS: 4")


    ## DEFINE AXES LEGENDS
    ax1_pos.legend([line_px,line_py,line_pz],[line_px.get_label(),line_py.get_label(),line_pz.get_label()])
    ax1_vel.legend([line_vx,line_vy,line_vz],[line_vx.get_label(),line_vy.get_label(),line_vz.get_label()])
    ax1_att.legend([line_ax,line_ay,line_az],[line_ax.get_label(),line_ay.get_label(),line_az.get_label()])
    ax1_omega.legend([line_wx,line_wy,line_wz],[line_wx.get_label(),line_wy.get_label(),line_wz.get_label()])
    ax1_ms.legend([line_ms1,line_ms2,line_ms3,line_ms4],[line_ms1.get_label(),line_ms2.get_label(),line_ms3.get_label(),line_ms4.get_label()])

    for ax in axes1:
        ax.set_xticks(np.linspace(0,buf_len,6))
        ax.set_xticklabels(np.linspace(-sec_hist,0,6))
        ax.set_xlabel("Seconds ago (Real Time)")
        ax.grid(True)
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.25), ncol=len(ax.lines))

    fig_0.tight_layout()




    fig_1 = plt.figure(1) # Initalize figure and axes
    ax2 = fig_1.add_subplot(111)

    k_ep = [] # Init empty arrays to be filled
    reward = []

    line_r, = ax2.plot(k_ep,reward) # Init line to be plotted


    def animate_reward(i,k_ep,reward):
        k_ep.append(k_ep_G)
        reward.append(reward_G)

        line_r.set_data(k_ep,reward)
        return line_r,
    


    flag = True # To be use for future dynamic scaling toggle

    # This function is called periodically from FuncAnimation
    def animate_dashboard(i,flag, pos_x,pos_y,pos_z,
        vel_x,vel_y,vel_z,
        att_x,att_y,att_z,
        omega_x,omega_y,omega_z):

        # States from shared Multiprocessing array
        x_x,x_y,x_z = state_G[1:4]
        qw,qx,qy,qz = state_G[4:8]
        vx,vy,vz = state_G[8:11]
        wx,wy,wz = state_G[11:14]

        if qw == 0: # Fix for zero-norm in quat error during initialization
            qw = 1

        R = Rotation.from_quat([qx,qy,qz,qw]) 
        yaw,pitch,roll = R.as_euler('zyx',degrees=True)
        

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


        ## ATTITUDE LINES
        att_x.append(roll)  
        att_x = att_x[-buf_len:] 
        line_ax.set_ydata(att_x) 

        att_y.append(pitch)  
        att_y = att_y[-buf_len:] 
        line_ay.set_ydata(att_y) 

        att_z.append(yaw)  
        att_z = att_z[-buf_len:] 
        line_az.set_ydata(att_z)


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

        
        ## DYNAMIC SCALING FOR FUTURE IMPLEMENTATION
        # max_omega = max(max(omega_x,omega_y,omega_z)) # Find absolute max omega element
        
        # if max_omega > 5 and flag == True:
        #     line_wy.axes.set_ylim([0,max_omega*1.5])
        #     fig_0.canvas.resize_event()
        #     flag = False
        # print("Test: ===========",reward_G.value)
        return line_px,line_py,line_pz,line_vx,line_vy,line_vz,line_ax,line_ay,line_az,line_wx,line_wy,line_wz

        


    # Set up plot to call animate() function periodically

    anim1 = animation.FuncAnimation(fig_0,
        animate_dashboard,
        fargs=(flag,
        pos_x,pos_y,pos_z,
        vel_x,vel_y,vel_z,
        att_x,att_y,att_z,
        omega_x,omega_y,omega_z),
        interval=interval,
        blit=True)

    # anim2 = animation.FuncAnimation(fig_1, 
    #     animate_reward, 
    #     fargs = (k_ep,reward),
    #     interval = interval,
    #     blit=True)

    plt.show()
    
