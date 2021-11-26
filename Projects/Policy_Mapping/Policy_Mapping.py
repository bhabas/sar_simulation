#!/usr/bin/env python3


import numpy as np
import time,os
import rospy
from crazyflie_msgs.msg import ImpactData,CtrlData
import math
import pandas as pd
import rospkg

import sys

crazyswarm_path = os.path.dirname(rospkg.RosPack().get_path('crazyflie_data'))
sys.path.insert(0, f'{crazyswarm_path}')

# cwd = os.getcwd()
# sys.path.insert(0, f'{cwd}/')
print(sys.path)




from crazyflie_rl.src.Crazyflie_env import CrazyflieEnv
print()
# from rospy.exceptions import ROSException

# os.system("clear")
# np.set_printoptions(precision=2, suppress=True)

# def runfunction(env,arr):
#     ## INITIALIALIZE LOGGING DATA
#     trial_prev = None
    

#     for vel,phi,d_ceiling,My,trial_num in arr:

#         if trial_num != trial_prev:
#             env.trial_name = f"Policy_Mapping--vel_{vel:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}--WL"
#             env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
#             env.logging_flag = True
#             env.create_csv(env.filepath)

#             env.k_run = 0
#             trial_prev = trial_num

#         env.policy = [vel,phi,d_ceiling,My]
            
#         ## INIT LAUNCH/FLIGHT CONDITIONS
#         phi_rad = np.radians(phi)
#         vy_d = 0 # [m/s]
#         env.vel_d = [vel*np.cos(phi_rad), vy_d, vel*np.sin(phi_rad)] # [m/s]

#         ## SEND MESSAGE FOR ALL NODES TO RESET TO DEFAULT VALUES
#         env.reset_flag = True
#         env.trialComplete_flag = False
#         env.RL_Publish() 

#         ## PRINT EPISODE DATA
#         print("=============================================")
#         print("STARTING Flip ")
#         print("=============================================")

#         # ============================
#         ##          Run 
#         # ============================

        

#         ## IF CONTROLLER FAILS, RELAUNCH IT
#         try:
#             rospy.wait_for_message("/ctrl_data",CtrlData,timeout=5.0)
#         except rospy.exceptions.ROSException:
#             env.launch_controller()
#             print("Resetting Controller")
#             time.sleep(2)
#             env.reset_pos()

          


#         ## RESET TO INITIAL STATE
#         env.reset_pos()
#         env.step('home') # Reset control vals and functionality to default vals
#         # time.sleep(0.65) # Time for CF to settle [Real-Time seconds]
        


#         ## RESET/UPDATE RUN CONDITIONS
#         repeat_run= False

#         start_time_rollout = env.getTime()
#         start_time_pitch = np.nan
#         start_time_impact = np.nan

#         ## RESET LOGGING CONDITIONS 
        
        
#         t_step = 0
#         t_prev = 0.0


        

        
#         flag = False # Ensures flip data recorded only once (Needs a better name)
#         flag2 = False # Ensures impact data recorded only once (Needs a better name)

#         ## STARTING LAUNCH HEIGHT
#         pos_z = env.h_ceiling - d_ceiling

#         env.step('sticky',ctrl_flag=1)                  # Enable sticky pads
#         env.step('pos',ctrl_flag=0)                     # Turn off pos control
#         env.step('vel',env.vel_d,ctrl_flag=1)           # Set desired vel
#         env.launch_IC(pos_z,env.vel_d[0],env.vel_d[2])  # Use Gazebo to impart desired vel 
#         env.step('moment',[0,My,0],ctrl_flag=1)

#         while True:



#             # ============================
#             ##      Pitch Recording 
#             # ============================
#             if (env.flip_flag == True and flag == False):
#                 start_time_pitch = env.getTime() # Starts countdown for when to reset run

#                 # Recieve flip moments from controller and then update class var to be sent out of /rl_data topic
#                 Mx_d = env.FM_flip[1] 
#                 My_d = env.FM_flip[2]
#                 Mz_d = env.FM_flip[3]
#                 env.M_d = [Mx_d,My_d,Mz_d] # [N*mm]

            
#                 flag = True # Turns on to make sure this only runs once per rollout

            
#             if ((env.impact_flag or env.body_contact) and flag2 == False):
#                 start_time_impact = env.getTime()
#                 flag2 = True

#             # ============================
#             ##    Termination Criteria 
#             # ============================
#             if (env.impact_flag or env.body_contact) and ((env.getTime()-start_time_impact) > 1.0):
#                 env.error_str = "Rollout Completed: Impact Timeout"
#                 print(env.error_str)

#                 env.runComplete_flag = True

#             # IF TIME SINCE TRIGGERED PITCH EXCEEDS [***]
#             elif env.flip_flag and ((env.getTime()-start_time_pitch) > (2.25)):
#                 env.error_str = "Rollout Completed: Pitch Timeout"
#                 print(env.error_str)

#                 env.runComplete_flag = True

#             # IF POSITION FALLS BELOW FLOOR HEIGHT
#             elif env.position[2] <= 0.0: # Note: there is a lag with this at high RTF
#                 env.error_str = "Rollout Completed: Falling Drone"
#                 print(env.error_str)

#                 env.runComplete_flag = True

#             # IF TIME SINCE RUN START EXCEEDS [***]
#             elif (env.getTime() - start_time_rollout) > (5.0):
#                 env.error_str = "Rollout Completed: Time Exceeded"
#                 print(env.error_str)

#                 env.runComplete_flag = True


#             # ============================
#             ##          Errors  
#             # ============================

#             ## IF NAN IS FOUND IN STATE VECTOR REPEAT RUN (Model collision Error)
#             if any(np.isnan(env.state_current)): 
#                 env.error_str = "Error: NAN found in state vector"
#                 print(env.error_str)
#                 repeat_run = True
#                 break

#             if np.abs(env.position[1]) >= 1.0: # If CF goes crazy it'll usually shoot out in y-direction
#                 env.error_str = "Error: Y-Position Exceeded"
#                 print(env.error_str)
#                 repeat_run = True
#                 break


#             # ============================
#             ##       Run Completion  
#             # ============================
#             if env.runComplete_flag==True:

#                 ## RUN DATA LOGGING
#                 env.append_csv_blank()
#                 env.append_IC()
#                 env.append_flip()
#                 env.append_impact()
#                 env.append_csv_blank()


#                 ## RESET/UPDATE RUN CONDITIONS
#                 env.runComplete_flag = False
#                 env.reset_flag = False
#                 env.error_str = ""

#                 env.clear_rollout_Data()

#                 env.k_run += 1

#                 break
                
#             t_prev = env.t   

#             ## =======  RUN COMPLETED  ======= ##
#             if repeat_run == True: # Runs when error detected
#                 env.relaunch_sim()

        





# if __name__ == '__main__':

#     ## INIT GAZEBO ENVIRONMENT
#     env = CrazyflieEnv(gazeboTimeout=False)
#     # env.launch_dashboard()

#     print("Environment done")
#     ## Home Test List
#     df = pd.read_csv("Projects/Policy_Mapping/PolicyMappingList.csv")
#     arr = df.to_numpy()


    

#     ## RUN TRIAL
#     env.RL_Publish() # Publish data to rl_data topic
#     time.sleep(3)


#     runfunction(env,arr)
    