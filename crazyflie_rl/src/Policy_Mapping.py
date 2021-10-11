#!/usr/bin/env python3


import numpy as np
import time,os
import rospy
from crazyflie_msgs.msg import ImpactData,CtrlData
import math
import pandas as pd



from Crazyflie_env import CrazyflieEnv
from rospy.exceptions import ROSException

os.system("clear")
np.set_printoptions(precision=2, suppress=True)

def runfunction(vel,phi,d_ceiling,My):

    ## INIT LAUNCH/FLIGHT CONDITIONS
    phi_rad = np.radians(phi)
    vy_d = 0 # [m/s]
    env.vel_d = [vel*np.cos(phi_rad), vy_d, vel*np.sin(phi_rad)] # [m/s]

    ## SEND MESSAGE FOR ALL NODES TO RESET TO DEFAULT VALUES
    env.reset_flag = True
    env.trialComplete_flag = False
    env.RL_Publish() 

    ## PRINT EPISODE DATA
    print("=============================================")
    print("STARTING Flip ")
    print("=============================================")

    # ============================
    ##          Run 
    # ============================

    try: # Use try block to catch raised exceptions and attempt rollout again

        ## IF CONTROLLER FAILS, RELAUNCH IT
        try:
            rospy.wait_for_message("/ctrl_data",CtrlData,timeout=5.0)
        except rospy.exceptions.ROSException:
            env.launch_controller()
            time.sleep(2)
            env.reset_pos()


        ## RESET TO INITIAL STATE
        env.reset_pos()
        env.step('home') # Reset control vals and functionality to default vals
        time.sleep(0.65) # Time for CF to settle [Real-Time seconds]
        


        ## RESET/UPDATE RUN CONDITIONS
        repeat_run= False

        start_time_rollout = env.getTime()
        start_time_pitch = np.nan
        start_time_impact = np.nan

        ## RESET LOGGING CONDITIONS 
        
        
        t_step = 0
        t_prev = 0.0


        

        
        flag = False # Ensures flip data recorded only once (Needs a better name)
        flag2 = False # Ensures impact data recorded only once (Needs a better name)


        pos_z = env.h_ceiling - d_ceiling

        env.step('pos',ctrl_flag=0)                     # Turn off pos control
        env.step('vel',env.vel_d,ctrl_flag=1)           # Set desired vel
        env.step('sticky',ctrl_flag=1)                  # Enable sticky pads
        env.launch_IC(pos_z,env.vel_d[0]+0.03,env.vel_d[2])   # Use Gazebo to impart desired vel with extra vx to ensure -OF_y when around zero
        env.step('moment',[0,-My*1e-3,0],ctrl_flag=1)

        env.step('sticky',ctrl_flag=0)


    except:
        pass




if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    # env.launch_dashboard()

    print("Environment done")
    ## Home Test List
    df = pd.read_csv("~/catkin_ws/src/crazyflie_simulation/crazyflie_data/data_collection/PolicyMappingList.csv")
    arr = df.to_numpy()


    for vel,phi,d_ceiling,My in arr:
        if np.isnan(vel):
            print("Trials are over")
            break

        runfunction(vel,phi,d_ceiling,My)

