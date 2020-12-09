#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation

# https://brushingupscience.com/2016/06/21/matplotlib-animations-the-easy-way/

def runGraph(STATE,K_EP,K_RUN,REWARD,REWARD_AVG,N_ROLLOUTS):

    # PARAMETERS
    buf_len = 300 # num datapoints in plot
    interval = 50 # Interval between plot frames [ms]
    sec_hist = buf_len*interval/1000 # Time shown on dashboard [s]
    buffer = list(range(0,buf_len)) # Number of points to display
    
    ## CREATE FIGURE FOR PLOTTING DASHBOARD
    fig_0 = plt.figure(0, figsize = (12,6))
    
    ax1_pos = plt.subplot2grid((2,3),(0,0))
    ax1_vel = plt.subplot2grid((2,3),(0,1))
    ax1_omega = plt.subplot2grid((2,3),(0,2))
    ax1_att = plt.subplot2grid((2,3),(1,0))
    ax1_ms = plt.subplot2grid((2,3),(1,1),colspan=2,rowspan=1)
    axes1 = [ax1_pos,ax1_vel,ax1_att,ax1_omega,ax1_ms]


    ## SET DASHBOARD TITLES
    ax1_pos.set_title('Position [m]')
    ax1_vel.set_title('Velocity [m/s]')
    ax1_att.set_title('Attitude [deg]')
    ax1_omega.set_title('Ang. Rate [rad/s]')
    ax1_ms.set_title('Motor speeds [rad/s]')

    ## SET DASHBOARD LABEL NAMES
    ax1_pos.set_ylabel("Pos. x,y,z [m]")
    ax1_vel.set_ylabel("Vel. x,y,z [m/s]")
    ax1_att.set_ylabel("Att. x,y,z [deg]")
    ax1_omega.set_ylabel("Ang. x,y,z [rad/s]")
    ax1_ms.set_ylabel("MS [rad/s]")


    ## SET  DASBOARD Y-LIMITS
    ax1_pos.set_ylim([-2,2])
    ax1_vel.set_ylim([-3,3])
    ax1_att.set_ylim([-200,200])
    ax1_omega.set_ylim([-30,30])
    ax1_ms.set_ylim([1500,2700])

    

    ## INIT BUFFER ARRAYS
    px_arr = [0]*buf_len
    py_arr = [0]*buf_len
    pz_arr = [0]*buf_len
    
    vx_arr = [0]*buf_len
    vy_arr = [0]*buf_len
    vz_arr = [0]*buf_len

    roll_arr = [0]*buf_len
    pitch_arr = [0]*buf_len
    yaw_arr = [0]*buf_len

    wx_arr = [0]*buf_len
    wy_arr = [0]*buf_len
    wz_arr = [0]*buf_len

    ms1_arr = [0]*buf_len
    ms2_arr = [0]*buf_len
    ms3_arr = [0]*buf_len
    ms4_arr = [0]*buf_len

    

    # Create a blank line. We will update the line in animate
    line_px, = ax1_pos.plot(buffer, px_arr,'b-',label="Pos. X")
    line_py, = ax1_pos.plot(buffer, py_arr,'g-',label="Pos. Y")
    line_pz, = ax1_pos.plot(buffer, pz_arr,'r-',label="Pos. Z")

    line_vx, = ax1_vel.plot(buffer, vx_arr,'b-',label="Vel. X")
    line_vy, = ax1_vel.plot(buffer, vy_arr,'g-',label="Vel. Y")
    line_vz, = ax1_vel.plot(buffer, vz_arr,'r-',label="Vel. Z")

    line_ax, = ax1_att.plot(buffer, roll_arr,'b-',label="Roll")
    line_ay, = ax1_att.plot(buffer, pitch_arr,'g-',label="Pitch")
    line_az, = ax1_att.plot(buffer, yaw_arr,'r-',label="Yaw")

    line_wx, = ax1_omega.plot(buffer, wx_arr,'b-',label='$\omega_x$')
    line_wy, = ax1_omega.plot(buffer, wy_arr,'g-',label='$\omega_y$')
    line_wz, = ax1_omega.plot(buffer, wz_arr,'r-',label='$\omega_z$')

    line_ms1, = ax1_ms.plot(buffer, ms1_arr,color = 'steelblue',linewidth = 1,label="MS: 1")
    line_ms2, = ax1_ms.plot(buffer, ms2_arr,'b-',linewidth = 1,label="MS: 2")
    line_ms3, = ax1_ms.plot(buffer, ms3_arr,'g-',linewidth = 1,label="MS: 3")
    line_ms4, = ax1_ms.plot(buffer, ms4_arr,'r-',linewidth = 1,label="MS: 4")


    ## DEFINE AXES LEGENDS
    ax1_pos.legend([line_px,line_py,line_pz],[line_px.get_label(),line_py.get_label(),line_pz.get_label()])
    ax1_vel.legend([line_vx,line_vy,line_vz],[line_vx.get_label(),line_vy.get_label(),line_vz.get_label()])
    ax1_att.legend([line_ax,line_ay,line_az],[line_ax.get_label(),line_ay.get_label(),line_az.get_label()])
    ax1_omega.legend([line_wx,line_wy,line_wz],[line_wx.get_label(),line_wy.get_label(),line_wz.get_label()])
    ax1_ms.legend([line_ms1,line_ms2,line_ms3,line_ms4],[line_ms1.get_label(),line_ms2.get_label(),line_ms3.get_label(),line_ms4.get_label()])

    for ax in axes1:
        ax.set_xticks(np.linspace(0,buf_len,6)) # These mark the second ticks
        ax.set_xticklabels(np.linspace(-sec_hist,0,6))
        ax.set_xlabel("Seconds ago (Sim Time)")

        ax.grid(True)
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.25), ncol=len(ax.lines)) # Places legend below plots

    fig_0.tight_layout() # Optimizes layout so no overlapping


    flag = True # To be use for future dynamic scaling toggle

    # This function is called periodically from FuncAnimation
    def animate_dashboard(i,flag, px_arr,py_arr,pz_arr,
        vx_arr,vy_arr,vz_arr,
        roll_arr,pitch_arr,yaw_arr,
        wx_arr,wy_arr,wz_arr):

        # States from shared Multiprocessing array
        x_x,x_y,x_z = STATE[1:4]
        qw,qx,qy,qz = STATE[4:8]
        vx,vy,vz = STATE[8:11]
        wx,wy,wz = STATE[11:14]
        

        if qw == 0: # Fix for zero-norm in quat error during initialization
            qw = 1
        R = Rotation.from_quat([qx,qy,qz,qw]) 
        yaw,pitch,roll = R.as_euler('zyx',degrees=True)
        

        ## POSITION LINES
        px_arr.append(x_x)  # Add y to list
        px_arr = px_arr[-buf_len:] # Limit y list to set number of items
        line_px.set_ydata(px_arr) # Update line with new Y values

        py_arr.append(x_y)  
        py_arr = py_arr[-buf_len:] 
        line_py.set_ydata(py_arr) 

        pz_arr.append(x_z)  
        pz_arr = pz_arr[-buf_len:] 
        line_pz.set_ydata(pz_arr)


        ## VELOCITY LINES
        vx_arr.append(vx)  
        vx_arr = vx_arr[-buf_len:] 
        line_vx.set_ydata(vx_arr) 

        vy_arr.append(vy)  
        vy_arr = vy_arr[-buf_len:] 
        line_vy.set_ydata(vy_arr) 

        vz_arr.append(vz)  
        vz_arr = vz_arr[-buf_len:] 
        line_vz.set_ydata(vz_arr) 


        ## ATTITUDE LINES
        roll_arr.append(roll)  
        roll_arr = roll_arr[-buf_len:] 
        line_ax.set_ydata(roll_arr) 

        pitch_arr.append(pitch)  
        pitch_arr = pitch_arr[-buf_len:] 
        line_ay.set_ydata(pitch_arr) 

        yaw_arr.append(yaw)  
        yaw_arr = yaw_arr[-buf_len:] 
        line_az.set_ydata(yaw_arr)


        ## ANG. RATE LINES
        wx_arr.append(wx)
        wx_arr = wx_arr[-buf_len:]
        line_wx.set_ydata(wx_arr)

        wy_arr.append(wy)
        wy_arr = wy_arr[-buf_len:]
        line_wy.set_ydata(wy_arr)

        wz_arr.append(wz)
        wz_arr = wz_arr[-buf_len:]
        line_wz.set_ydata(wz_arr)



        
        ## DYNAMIC SCALING FOR FUTURE IMPLEMENTATION
        # max_omega = max(max(omega_x,omega_y,omega_z)) # Find absolute max omega element
        
        # if max_omega > 5 and flag == True:
        #     line_wy.axes.set_ylim([0,max_omega*1.5])
        #     fig_0.canvas.resize_event()
        #     flag = False
        # print("Test: ===========",reward_G.value)
        return  line_px,line_py,line_pz, \
                line_vx,line_vy,line_vz, \
                line_ax,line_ay,line_az, \
                line_wx,line_wy,line_wz, 
                








    ## CREATE FIGURE FOR REWARD PLOT
    fig_1 = plt.figure(1,figsize=(6,3)) # Initalize figure and axes

    ax2 = fig_1.add_subplot(111)
    ax2.set_xlabel("Episode")
    ax2.set_ylabel("Reward")
    ax2.set_xlim([-2,40])
    ax2.set_ylim([0,22])

    ax2.grid(True)
    ax2.set_title(f'Rollouts: {N_ROLLOUTS.value*2} | Episode: {K_EP.value} Run: {K_RUN.value} (Completed) ')


    ## INIT ARRAYS TO BE FILLED
    k_ep = [0] # Need zero value just to be able to call [-1]
    k_run = [0] 
    reward = [0]

    k_ep_ravg = [0] # k_ep grows on each run iteration so a new array was created to match reward_avg size
    reward_avg = [0]
    
    scat_r = ax2.scatter([],[],marker='_', color = "black", alpha = 0.5) # Reward scatter
    scat_ra = ax2.scatter([],[],marker='o',color = 'red') # Reward_average scatter


    def animate_reward(i,k_ep,k_run,k_ep_ravg):

        if k_ep[-1] != K_EP.value: # if new episode append values and plot avg reward
            reward_avg.append(REWARD_AVG.value)
            k_ep_ravg.append(K_EP.value-1)
            scat_ra.set_offsets(np.c_[k_ep_ravg[1:],reward_avg[1:]])
        
        if k_run[-1] != K_RUN.value: # if new run append values and plot reward
            k_ep.append(K_EP.value)
            reward.append(REWARD.value)
            k_run.append(K_RUN.value)
            scat_r.set_offsets(np.c_[k_ep[1:],reward[1:]])
            ax2.set_title(f'Rollouts: {N_ROLLOUTS.value*2} | Episode: {K_EP.value} Run: {K_RUN.value} (Completed) ')


        return scat_r,scat_ra
 

    anim1 = animation.FuncAnimation(fig_0,
        animate_dashboard,
        fargs=(flag,
        px_arr,py_arr,pz_arr,
        vx_arr,vy_arr,vz_arr,
        roll_arr,pitch_arr,yaw_arr,
        wx_arr,wy_arr,wz_arr),
        interval=interval,
        blit=True)

    anim2 = animation.FuncAnimation(fig_1, 
        animate_reward, 
        fargs = (k_ep,k_run,k_ep_ravg),
        interval = interval,
        blit=False)

    plt.show()

    ## STRUCTURE: FuncAnimation
    # Initialize figure and axes 
    # Create an empty array and plot it
    # Create a function that appends to arrays whenever it's called
    #       and returns the line/scatter objects
    #
    # Use FuncAnimation to call that function with it's fargs inputs
    # Note: If only one input include comma after it e.g. fargs=(temp1,)
    #
    # interval: time between frames
    # blit: improves performance by only redrawing new objects (Causes issues with display though)
    #       blit = False, will remove issues but lowers frame rate

    