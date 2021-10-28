#! /usr/bin/env python3

import os,time
import rospy

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation



from dashboard_node import DashboardNode
## EXAMPLE FOR MATPLOTLIB ANIMATIONS:
# https://brushingupscience.com/2016/06/21/matplotlib-animations-the-easy-way/

# NOTE: This will likely be scrapped down the line for PyQtGraph which is faster

os.system("clear")
DashNode=DashboardNode()



def startDashboard():
    
    # PARAMETERS
    sec_hist = 10 # Time shown on dashboard [s]
    frame_interval = 100 # Interval between plot frames [ms]
    buf_len = int(sec_hist/frame_interval * 1000) # num datapoints in plot

    buffer = list(range(0,buf_len)) # Number of points to display
    


    ## CREATE FIGURE FOR PLOTTING DASHBOARD
    fig_0 = plt.figure(0, figsize = (12,6))
    
    ax1_pos = plt.subplot2grid((2,3),(0,0))
    ax1_vel = plt.subplot2grid((2,3),(0,1))
    ax1_omega = plt.subplot2grid((2,3),(0,2))
    ax1_FM = plt.subplot2grid((2,3),(1,0))
    ax1_ms = plt.subplot2grid((2,3),(1,1),colspan=2,rowspan=1)
    axes1 = [ax1_pos,ax1_vel,ax1_FM,ax1_omega,ax1_ms]


    ## SET DASHBOARD TITLES
    ax1_pos.set_title('Position [m]')
    ax1_vel.set_title('Velocity [m/s]')
    ax1_FM.set_title('Flip Magnitude [N*mm]')
    ax1_omega.set_title('Ang. Rate [rad/s]')
    ax1_ms.set_title('Motor speeds [rad/s]')

    ## SET DASHBOARD LABEL NAMES
    ax1_pos.set_ylabel("Pos. x,y,z [m]")
    ax1_vel.set_ylabel("Vel. x,y,z [m/s]")
    ax1_FM.set_ylabel(" M [N*mm] | F [0.1*N]")
    ax1_omega.set_ylabel("Ang. x,y,z [rad/s]")
    ax1_ms.set_ylabel("MS [rad/s]")


    ## SET  DASBOARD Y-LIMITS
    ax1_pos.set_ylim([-1,7])
    ax1_vel.set_ylim([-1,7])
    ax1_FM.set_ylim([0,10])
    ax1_omega.set_ylim([-30,30])
    ax1_ms.set_ylim([0,2700])

    

    ## INIT BUFFER ARRAYS
    px_arr = [0]*buf_len
    py_arr = [0]*buf_len
    pz_arr = [0]*buf_len
    
    vx_arr = [0]*buf_len
    vy_arr = [0]*buf_len
    vz_arr = [0]*buf_len

    F_d_arr = [0]*buf_len
    My_d_arr = [0]*buf_len
    My_arr = [0]*buf_len

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

    line_F, = ax1_FM.plot(buffer, F_d_arr,'b-',label="F_thrust")
    line_My_d, = ax1_FM.plot(buffer, My_d_arr,'g-',label="My_flip [N*mm]")
    line_My, = ax1_FM.plot(buffer, My_arr,'r-',label="My [N*mm]")

    line_wx, = ax1_omega.plot(buffer, wx_arr,'b-',label='$\omega_x$')
    line_wy, = ax1_omega.plot(buffer, wy_arr,'g-',label='$\omega_y$')
    line_wz, = ax1_omega.plot(buffer, wz_arr,'r-',label='$\omega_z$')

    line_ms1, = ax1_ms.plot(buffer, ms1_arr,color = 'steelblue',linewidth = 1.5,label="MS: 1")
    line_ms2, = ax1_ms.plot(buffer, ms2_arr,'b-',linewidth = 1.5,label="MS: 2")
    line_ms3, = ax1_ms.plot(buffer, ms3_arr,'g-',linewidth = 1.5,label="MS: 3")
    line_ms4, = ax1_ms.plot(buffer, ms4_arr,'r-',linewidth = 1.5,label="MS: 4")


    ## DEFINE AXES LEGENDS
    ax1_pos.legend([line_px,line_py,line_pz],[line_px.get_label(),line_py.get_label(),line_pz.get_label()])
    ax1_vel.legend([line_vx,line_vy,line_vz],[line_vx.get_label(),line_vy.get_label(),line_vz.get_label()])
    ax1_FM.legend([line_F,line_My_d,line_My],[line_F.get_label(),line_My_d.get_label(),line_My.get_label()])
    ax1_omega.legend([line_wx,line_wy,line_wz],[line_wx.get_label(),line_wy.get_label(),line_wz.get_label()])
    ax1_ms.legend([line_ms1,line_ms2,line_ms3,line_ms4],[line_ms1.get_label(),line_ms2.get_label(),line_ms3.get_label(),line_ms4.get_label()])

    for ax in axes1:
        ax.set_xticks(np.linspace(0,buf_len,6)) # These mark the second ticks
        ax.set_xticklabels(np.linspace(-sec_hist,0,6))
        ax.set_xlabel("Seconds ago (Sim Time)")

        ax.grid(True)
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.25), ncol=len(ax.lines)) # Places legend below plots

    fig_0.tight_layout() # Optimizes layout so no overlapping


    
    # This function is called periodically from FuncAnimation
    def animate_dashboard(i, px_arr,py_arr,pz_arr,
        vx_arr,vy_arr,vz_arr,
        F_d_arr,My_d_arr,My_arr,
        wx_arr,wy_arr,wz_arr,
        ms1_arr,ms2_arr,ms3_arr,ms4_arr):

        # States from class
        x_x,x_y,x_z = DashNode.state_current[1:4]
        qw,qx,qy,qz = DashNode.state_current[4:8]
        vx,vy,vz = DashNode.state_current[8:11]
        wx,wy,wz = DashNode.state_current[11:14]

        ms1,ms2,ms3,ms4 = DashNode.MS
        My,My_flip,F_thrust = np.abs(DashNode.FM[2]),np.abs(DashNode.FM_flip[2]),DashNode.FM[0]*1e1
        

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
        F_d_arr.append(F_thrust)  
        F_d_arr = F_d_arr[-buf_len:] 
        line_F.set_ydata(F_d_arr) 

        My_d_arr.append(My_flip)  
        My_d_arr = My_d_arr[-buf_len:] 
        line_My_d.set_ydata(My_d_arr) 

        My_arr.append(My)  
        My_arr = My_arr[-buf_len:] 
        line_My.set_ydata(My_arr)


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

        ## MOTORSPEED LINES
        lineoffset = 25
        ms1_arr.append(ms1+lineoffset) # Added offsets so indiv. motorspeeds are visible on plot
        ms1_arr = ms1_arr[-buf_len:]
        line_ms1.set_ydata(ms1_arr)

        ms2_arr.append(ms2-lineoffset)
        ms2_arr = ms2_arr[-buf_len:]
        line_ms2.set_ydata(ms2_arr)

        ms3_arr.append(ms3+lineoffset)
        ms3_arr = ms3_arr[-buf_len:]
        line_ms3.set_ydata(ms3_arr)

        ms4_arr.append(ms4-lineoffset)
        ms4_arr = ms4_arr[-buf_len:]
        line_ms4.set_ydata(ms4_arr)




        return  line_px,line_py,line_pz, \
                line_vx,line_vy,line_vz, \
                line_F,line_My_d,line_My, \
                line_wx,line_wy,line_wz, \
                line_ms1,line_ms2,line_ms3,line_ms4
                    

 
    
    anim1 = animation.FuncAnimation(fig_0,
        animate_dashboard,
        fargs=(
        px_arr,py_arr,pz_arr,
        vx_arr,vy_arr,vz_arr,
        F_d_arr,My_d_arr,My_arr,
        wx_arr,wy_arr,wz_arr,
        ms1_arr,ms2_arr,ms3_arr,ms4_arr),
        interval=frame_interval,
        blit=True)

    plt.show()



if __name__ == "__main__":
    startDashboard()