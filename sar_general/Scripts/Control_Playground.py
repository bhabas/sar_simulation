#!/usr/bin/env python3
import threading,os
import rospy
import numpy as np

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = "\033[34m"  
RESET = "\033[0m"  # Reset to default color
       
def cmd_send(env,logName):

    while True:
        # Converts input number into action name
        cmd_dict = {
            0:'Ctrl_Reset',
            1:'Pos',
            2:'Vel',

            5:'Stop',
            7:'Ang_Accel',
            8:'Policy',
            9:'Plane_Pose',

            10:'P2P_traj',
            11:'Global_Vel_traj',
            12:'Rel_Vel_traj',
            13:'Impact_traj',

            20:'Tumble_Detect',
            21:'Load_Params',
            22:'Start_Logging',
            23:'Cap_Logging',
            24:'Arm_Quad',

            90:'GZ_Pose_Reset',
            91:'GZ_Const_Vel_Traj',
            92:'GZ_StickyPads',
        }

        try:
            print("========== Command Types ==========")
            print("0: Ctrl_Reset \t1: Pos \t2: Vel \t5: Stop")
            print("7: Ang_Accel \t8: Policy \t9: Plane_Pose")
            print("10: P2P_traj \t11: Vel_traj \t12: Impact_traj")
            print("20: Tumble_Detect \t21: Load_Params \t22: Start_Logging \t23: Cap_Logging \t24: Safe_Mode")
            print("90: GZ_Pose_Reset \t91: GZ_Const_Vel_Traj \t92: GZ_StickyPads")
 
            val = env.userInput("\nCmd: ",int)
            print()
            action = cmd_dict[val]
            

            ## CONTROLLER RESET
            if action=='Ctrl_Reset': # Execute Ctrl_Reset or Stop action
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Reset controller to default values\n")

                env.sendCmd('GZ_StickyPads',cmd_vals,0)
                env.sendCmd(action,cmd_vals,cmd_flag)
            

            elif action=='Pos':
                cmd_vals = env.userInput("Set desired position values (x,y,z): ",float)
                cmd_flag = env.userInput("Pos control On/Off (1,0): ",int)
                env.sendCmd(action,cmd_vals,cmd_flag)

            elif action=='Vel':
                cmd_vals = env.userInput("Set desired velocity values (x,y,z): ",float)
                cmd_flag = env.userInput("Vel control On/Off (1,0): ",int)
                env.sendCmd(action,cmd_vals,cmd_flag)

            elif action=='Stop': # Execute Ctrl_Reset or Stop action
                cmd_vals = [0,0,0]
                cmd_flag = 1
                env.sendCmd(action,cmd_vals,cmd_flag)

            elif action=='Ang_Accel':
                cmd_vals = env.userInput("Set desired angular acceleration values (x,y,z): ",float)
                cmd_flag = env.userInput("Ang_Accel control On/Off (1,0): ",int)
                env.sendCmd(action,cmd_vals,cmd_flag)

            elif action=='Policy':
                cmd_vals = env.userInput("Set desired (Tau,My_d) Policy: ",float)
                cmd_vals.append(0) # Append extra value to match framework
                cmd_flag = 1
                env.sendCmd(action,cmd_vals,cmd_flag)


            elif action=='Plane_Pose':
                cmd_vals = env.userInput("Set desired position values (x,y,z): ",float)
                cmd_flag = env.userInput("Set desired plane angle [deg]: ",float)
                env._setPlanePose(cmd_vals,cmd_flag)


            ## ========== TRAJECTORY FUNCTIONS ==========

            elif action=='P2P_traj':
                x_d = env.userInput("Desired position (x,y,z):",float)
                env.sendCmd('P2P_traj',cmd_vals=[env.r_B_O[0],x_d[0],env.TrajAcc_Max[0]],cmd_flag=0)
                env.sendCmd('P2P_traj',cmd_vals=[env.r_B_O[1],x_d[1],env.TrajAcc_Max[1]],cmd_flag=1)
                env.sendCmd('P2P_traj',cmd_vals=[env.r_B_O[2],x_d[2],env.TrajAcc_Max[2]],cmd_flag=2)

            elif action=='Global_Vel_traj':

                ## GET GLOBAL VEL CONDITIONS 
                V_mag,V_angle = env.userInput("Flight Velocity (V_mag,V_angle):",float)

                ## CALC GLOBAL VELOCITIES
                Vx = V_mag*np.cos(np.radians(V_angle))
                Vy = 0
                Vz = V_mag*np.sin(np.radians(V_angle))
                V_B_O = [Vx,Vy,Vz]

                ## EXECUTE TRAJECTORY
                env.sendCmd('Global_Vel_traj',cmd_vals=[env.r_B_O[0],V_B_O[0],env.TrajAcc_Max[0]],cmd_flag=0)
                env.sendCmd('Global_Vel_traj',cmd_vals=[env.r_B_O[2],V_B_O[2],env.TrajAcc_Max[2]],cmd_flag=2)

            elif action=='Rel_Vel_traj':

                ## GET RELATIVE VEL CONDITIONS 
                V_mag,V_angle = env.userInput("Flight Velocity (V_mag,V_angle):",float)

                ## CALC RELATIVE VELOCITIES
                V_tx = V_mag*np.cos(np.radians(V_angle))
                V_ty = 0
                V_perp = V_mag*np.sin(np.radians(V_angle))

                ## CALCULATE GLOBAL VELOCITIES
                V_B_O = env.R_PW(np.array([V_tx,V_ty,V_perp]),env.Plane_Angle_rad)

                ## EXECUTE TRAJECTORY
                env.sendCmd('Global_Vel_traj',cmd_vals=[env.r_B_O[0],V_B_O[0],env.TrajAcc_Max[0]],cmd_flag=0)
                env.sendCmd('Global_Vel_traj',cmd_vals=[env.r_B_O[2],V_B_O[2],env.TrajAcc_Max[2]],cmd_flag=2)

            elif action=='Impact_traj':

                ## GET RELATIVE VEL CONDITIONS 
                V_mag,V_angle = env.userInput("Flight Velocity (V_mag,V_angle):",float)

                ## CALC RELATIVE VELOCITIES
                V_tx = V_mag*np.cos(np.radians(V_angle))
                V_ty = 0
                V_perp = V_mag*np.sin(np.radians(V_angle))
                V_B_P = np.array([V_tx,V_ty,V_perp])

                ## CALCULATE GLOBAL VELOCITIES
                V_B_O = env.R_PW(V_B_P,env.Plane_Angle_rad)

                ## POS VELOCITY CONDITIONS MET
                r_B_O = env.startPos_ImpactTraj(V_B_P,Acc=None,Tau_CR_start=None)

                print(YELLOW,f"Start Position: ({r_B_O[0]:.2f},{env.r_B_O[1]:.2f},{r_B_O[2]:.2f})",RESET)
                str_input = env.userInput("Approve start position (y/n): ",str)
                if str_input == 'y':
                    env.sendCmd('P2P_traj',cmd_vals=[env.r_B_O[0],r_B_O[0],env.TrajAcc_Max[0]],cmd_flag=0)
                    env.sendCmd('P2P_traj',cmd_vals=[env.r_B_O[1],r_B_O[1],env.TrajAcc_Max[1]],cmd_flag=1)
                    env.sendCmd('P2P_traj',cmd_vals=[env.r_B_O[2],r_B_O[2],env.TrajAcc_Max[2]],cmd_flag=2)
                else:
                    continue

                str_input = env.userInput("Approve flight (y/n): ",str)
                if str_input == 'y':
                    env.sendCmd('Global_Vel_traj',cmd_vals=[env.r_B_O[0],V_B_O[0],env.TrajAcc_Max[0]],cmd_flag=0)
                    env.sendCmd('Global_Vel_traj',cmd_vals=[env.r_B_O[2],V_B_O[2],env.TrajAcc_Max[2]],cmd_flag=2)
                else:
                    continue

            
            ## ========== SYSTEM FUNCTIONS ==========
            elif action=='Tumble_Detect': # Turn on Tumble detection

                cmd_vals = [0,0,0]
                cmd_flag = env.userInput("Tumble Detection On/Off (1,0): ",int)
                env.sendCmd('Tumble',cmd_vals,cmd_flag)

            elif action=='Load_Params': # Updates gain values from config file
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Reset ROS Parameters\n")

                env.setParams()
                env.sendCmd(action,cmd_vals,cmd_flag)


            elif action=='Start_Logging':
                env.startLogging(logName)

            elif action=='Cap_Logging':
                env.capLogging(logName)

            elif action=='Arm_Quad':
                cmd_vals = [0,0,0]
                cmd_flag = env.userInput("Arm Quad On/Off (1,0): ",int)
                env.sendCmd(action,cmd_vals,cmd_flag)

            

            ## ========== GAZEBO FUNCTIONS ==========
            elif action=='GZ_StickyPads':
                cmd_vals = [0,0,0]
                cmd_flag = env.userInput("Turn sticky pads On/Off (1,0): ",int)
                print()

                env.sendCmd(action,cmd_vals,cmd_flag)



            elif action=='GZ_Const_Vel_Traj':


                ## GET INPUT VALUES
                V_B_O_mag,V_B_O_angle = env.userInput("Flight Velocity (V_B_O_mag,V_B_O_angle):",float)

                ## DEFINE CARTESIAN VELOCITIES
                V_B_O_angle = np.radians(V_B_O_angle)
                V_B_O = [V_B_O_mag*np.cos(V_B_O_angle),
                         0,
                         V_B_O_mag*np.sin(V_B_O_angle)]

                ## ESTIMATE IMPACT POINT
                env.GZ_VelTraj(env.r_B_O,V_B_O)
                env.pausePhysics(False)
                
                    
            elif action == 'GZ_Pose_Reset':
                print("Reset Pos/Vel -- Sticky off -- Controller Reset\n")
                env.resetPose()
                env.pausePhysics(pause_flag=False)



            else:
                print("Please try another command")

        except (ValueError, KeyError,TypeError,AttributeError):
            print('\033[93m' + "INVALID INPUT: Try again" + '\x1b[0m')
            continue


if __name__ == '__main__':
    from sar_env import SAR_Sim_Interface
    ## INIT GAZEBO ENVIRONMENT
    env = SAR_Sim_Interface(GZ_Timeout=False)
    env.pausePhysics(False)

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    logName = f"Control_Playground--trial_{int(trial_num):02d}--{env.SAR_Config}.csv"

    env.createCSV(logName)
    cmd_thread = threading.Thread(target=cmd_send,args=(env,logName))
    cmd_thread.start()   

    rospy.spin()