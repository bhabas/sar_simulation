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
    


    ## CREATE FIGURE FOR REWARD PLOT
    fig_1 = plt.figure(1,figsize=(6,3)) # Initalize figure and axes

    ax2 = fig_1.add_subplot(111)
    ax2.set_xlabel("Episode")
    ax2.set_ylabel("Reward")
    ax2.set_xlim([-2,40])
    ax2.set_ylim([-2,150])

    ax2.grid(True)
    
    


    ## INIT EMPTY ARRAYS TO BE FILLED
    k_ep_arr = [np.nan] # Need value just to be able to call [-1]
    k_run_arr = [np.nan] 
    reward_arr = [np.nan]
    reward_avg_arr = [np.nan]

    
    ax2.set_title(f'Rollouts: {DashNode.n_rollouts} | Episode: {DashNode.k_ep} Run: {DashNode.k_run} (Completed) ')
    scat_reward = ax2.scatter([],[],marker='_', color = "black", alpha = 0.5) # Reward scatter
    scat_reward_avg = ax2.scatter([],[],marker='o',color = "red")
    fig_1.tight_layout()
    
    def animate_reward(i,k_ep_arr,k_run_arr,reward_arr): ## This runs in continuous loop
    
        if k_run_arr[-1] != DashNode.k_run: # If k_run changes

            ## APPEND CLASS VARIABLES TO ARRAY
            k_run_arr.append(DashNode.k_run)
            k_ep_arr.append(DashNode.k_ep)
            reward_arr.append(DashNode.reward)
            

            ## APPEND R_AVG VALUES ONLY ON LAST RUN
            if k_run_arr[-1] == (DashNode.n_rollouts-1): # If k_ep changes
                reward_avg_arr.append(DashNode.reward_avg)
            else: ## OTHERWISE APPEND NON-PLOTTABLE NAN
                reward_avg_arr.append(np.nan)

            ## PLOT APPENDED ARRAYS
            scat_reward.set_offsets(np.c_[k_ep_arr[1:],reward_arr[1:]])
            scat_reward_avg.set_offsets(np.c_[k_ep_arr[1:],reward_avg_arr[1:]])

            ## UPDATE TITLE AND FIG FORMATTING
            ax2.set_title(f'Rollouts: {DashNode.n_rollouts} | Episode: {DashNode.k_ep} Run: {DashNode.k_run} (Completed) ')
            fig_1.tight_layout()
        
        return scat_reward,scat_reward_avg

 
    
    
    anim2 = animation.FuncAnimation(fig_1, 
        animate_reward, 
        fargs=(k_ep_arr,k_run_arr,reward_arr),
        interval = frame_interval,
        blit=False)

    plt.show()



if __name__ == "__main__":
    startDashboard()