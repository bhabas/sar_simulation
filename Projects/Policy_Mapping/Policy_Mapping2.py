#!/usr/bin/env python3

## STANDARD IMPORTS
import time
import os
import rospy
import math

## PACKAGE IMPORTS
import numpy as np
import pandas as pd

import sys
cwd = os.getcwd()
sys.path.insert(0, f'{cwd}/')

## CUSTOM IMPORTS
from crazyflie_msgs.msg import ImpactData,CtrlData
from rospy.exceptions import ROSException
from crazyflie_rl.src.Crazyflie_env import CrazyflieEnv







os.system("clear")
np.set_printoptions(precision=2, suppress=True)


## INIT GAZEBO ENVIRONMENT
env = CrazyflieEnv(gazeboTimeout=False)
# env.launch_statedashboard()

print("Environment done")

## IMPORT TEST LIST
df = pd.read_csv("Projects/Policy_Mapping/PolicyMappingList.csv")
arr = df.to_numpy()

input()

for vel,phi,d_ceiling,My in arr:

    ## INIT LAUNCH/FLIGHT CONDITIONS
    phi_rad = np.radians(phi)
    vy_d = 0 # [m/s]
    env.vel_d = [vel*np.cos(phi_rad), vy_d, vel*np.sin(phi_rad)] # [m/s]


    ## PRINT EPISODE DATA
    print("=============================================")
    print(f"EXECUTING FLIP: vel: {vel} \t phi: {phi} \t My: {My}")
    print("=============================================")


    ## IF CONTROLLER FAILS, RELAUNCH IT
    for attempt in range(5):
        try:
            rospy.wait_for_message("/ctrl_data",CtrlData,timeout=5.0)
            break
        except rospy.exceptions.ROSException:
            env.launch_controller()
            print(f"Resetting Controller: Attempt({attempt})")
            time.sleep(2)
            env.reset_pos()


    ## RESET TO INITIAL STATE
    env.reset_pos()
    env.step('home') # Reset control vals and functionality to default vals
        

    ## RESET/UPDATE RUN CONDITIONS
    repeat_run= False

    start_time_rollout = env.getTime()
    start_time_pitch = np.nan
    start_time_impact = np.nan

    flag = False # Ensures flip data recorded only once (Needs a better name)
    flag2 = False # Ensures impact data recorded only once (Needs a better name)

    ## STARTING LAUNCH HEIGHT
    pos_z = env.h_ceiling - d_ceiling

    env.step('sticky',ctrl_flag=1)                  # Enable sticky pads
    env.step('pos',ctrl_flag=0)                     # Turn off pos control
    env.step('vel',env.vel_d,ctrl_flag=1)           # Set desired vel
    env.launch_IC(pos_z,env.vel_d[0],env.vel_d[2])  # Use Gazebo to impart desired vel at pos
    env.step('moment',[0,My*1e-3,0],ctrl_flag=1)         # Execute desired moment

    while True:

        # ============================
        ##      Pitch Recording 
        # ============================
        if (env.flip_flag == True and flag == False):
            start_time_pitch = env.getTime() # Starts countdown for when to reset run

            # Recieve flip moments from controller and then update class var to be sent out of /rl_data topic
            Mx_d = env.FM_flip[1] 
            My_d = env.FM_flip[2]
            Mz_d = env.FM_flip[3]
            env.M_d = [Mx_d,My_d,Mz_d] # [N*mm]

        
            flag = True # Turns on to make sure this only runs once per rollout

        
        if ((env.impact_flag or env.body_contact) and flag2 == False):
            start_time_impact = env.getTime()
            flag2 = True




        # ============================
        ##    Termination Criteria 
        # ============================
        if (env.impact_flag or env.body_contact) and ((env.getTime()-start_time_impact) > 1.0):
            env.error_str = "Rollout Completed: Impact Timeout"
            print(env.error_str)

            env.runComplete_flag = True

        # IF TIME SINCE TRIGGERED PITCH EXCEEDS [***]
        elif env.flip_flag and ((env.getTime()-start_time_pitch) > (2.25)):
            env.error_str = "Rollout Completed: Pitch Timeout"
            print(env.error_str)

            env.runComplete_flag = True

        # IF POSITION FALLS BELOW FLOOR HEIGHT
        elif env.position[2] <= 0.0: 
            env.error_str = "Rollout Completed: Falling Drone"
            print(env.error_str)

            env.runComplete_flag = True

        # IF TIME SINCE RUN START EXCEEDS [***]
        elif (env.getTime() - start_time_rollout) > (5.0):
            env.error_str = "Rollout Completed: Time Exceeded"
            print(env.error_str)

            env.runComplete_flag = True

        # ============================
        ##          Errors  
        # ============================

        ## IF NAN IS FOUND IN STATE VECTOR REPEAT RUN (Model collision Error)
        if any(np.isnan(env.state_current)): 
            env.error_str = "Error: NAN found in state vector"
            print(env.error_str)
            repeat_run = True
            break

        if np.abs(env.position[1]) >= 1.0: # If CF goes crazy it'll usually shoot out in y-direction
            env.error_str = "Error: Y-Position Exceeded"
            print(env.error_str)
            repeat_run = True
            break

        # ============================
        ##       Run Completion  
        # ============================
        if env.runComplete_flag==True:
            ## RESET/UPDATE RUN CONDITIONS
            env.runComplete_flag = False
            env.reset_flag = False
            env.error_str = ""

            env.clear_rollout_Data()

            ## LOGGING
            break

