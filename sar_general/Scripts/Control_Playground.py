#!/usr/bin/env python3
import threading,os
import rospy
import time
import numpy as np

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = "\033[34m"  
RESET = "\033[0m"  # Reset to default color
       
def cmd_send(env):

    while True:
        # Converts input number into action name

        command_handlers = {
            0: env.handle_Ctrl_Reset,
            1: env.handle_Pos_Cmd,
            2: env.handle_Vel_Cmd,
            5: env.handle_Stop,
            7: env.handle_Ang_Accel,
            8: env.handle_Policy,
            9: env.handle_Plane_Pose,
            10: env.handle_P2P_traj,
            11: env.handle_Global_Vel_traj,
            12: env.handle_Rel_Vel_traj,
            13: env.handle_Impact_traj,
            20: env.handle_Tumble_Detect,
            21: env.handle_Load_Params,
            22: env.handle_Start_Logging,
            23: env.handle_Cap_Logging,
            24: env.handle_Arm_Quad,
            30: env.handle_Thrust_CMD,
            31: env.handle_Motor_CMD,
            90: env.handle_GZ_Pose_Reset,
            91: env.handle_GZ_StickyPads,
            92: env.handle_GZ_Global_Vel_traj,
            93: env.handle_GZ_Rel_Vel_traj,

        }
        
        try:
            print("========== Command Types ==========")
            print("0: Ctrl_Reset  7: Ang_Accel    10: P2P_traj          20: Tumble_Detect    24: Arm_Quad    90: GZ_Pose_Reset")
            print("1: Pos         8: Policy       11: Global_Vel_traj   21: Load_Params      30: Thrust_CMD  91: GZ_StickyPads")
            print("2: Vel         9: Plane_Pose   12: Rel_Vel_traj      22: Start_Logging    31: Motor_CMD   92: GZ_Global_Vel_traj")
            print("5: Stop                        13: Impact_traj       23: Cap_Logging                      93: GZ_Rel_Vel_traj")


            cmd = env.userInput("\nCmd: ",int)
            if cmd in command_handlers:
                command_handlers[cmd]()
            else:
                print("Invalid Command: Try again")
        
        except Exception as e:
            print(f"Error: {e}")
            print(YELLOW + "INVALID INPUT: Try again" + RESET)
            continue


if __name__ == '__main__':
    from sar_env import SAR_Sim_Interface
    ## INIT GAZEBO ENVIRONMENT
    env = SAR_Sim_Interface(GZ_Timeout=False)
    env.pausePhysics(False)

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    env.Log_Name = f"Control_Playground--trial_{int(trial_num):02d}--{env.SAR_Config}.csv"

    env.createCSV()
    cmd_thread = threading.Thread(target=cmd_send,args=(env,))
    cmd_thread.start()   

    rospy.spin()