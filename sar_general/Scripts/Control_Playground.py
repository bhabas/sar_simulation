#!/usr/bin/env python3
import threading,os
import rospy
import numpy as np

       
def cmd_send(env,logName):
    while True:
        # Converts input number into action name
        cmd_dict = {
            0:'Ctrl_Reset',
            1:'Pos',
            2:'Vel',
            3:'Yaw',
            5:'Stop',
            7:'Ang_Accel',
            8:'Policy',

            10:'P2P_traj',
            11:'Vel_traj',
            12:'Impact_traj',

            20:'Tumble',
            21:'Load_Params',
            22:'Start_Logging',
            23:'Cap_Logging',

            90:'GZ_Pose_Reset',
            91:'GZ_Const_Vel_Traj',
            92:'GZ_StickyPads',
            93:'GZ_Plane_Pose',
        }

        try:
            print("========== Command Types ==========")
            print("0:Ctrl_Reset, \t1:Pos, \t\t2:Vel, \t3:Yaw, \t5:Stop, \t8:Policy")
            print("10:P2P_traj, \t11:Vel_traj, \t12:Impact_traj,")
            print("20:Tumble, \t21:Load_Params, 22:Start_Logging, 23:Cap_Logging,")
            print("90:GZ_Pose_Reset, \t91:GZ_Const_Vel_Traj, \t92:GZ_StickyPads, \t93:GZ_Plane_Pose")
            val = env.userInput("\nCmd: ",int)
            print()
            action = cmd_dict[val]
            


            if action=='Ctrl_Reset': # Execute Ctrl_Reset or Stop action
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Reset controller to default values\n")

                env.sendCmd('GZ_StickyPads',cmd_vals,0)
                env.sendCmd(action,cmd_vals,cmd_flag)
            
            elif action=='Start_Logging':
                env.startLogging(logName)

            elif action=='Cap_Logging':
                env.capLogging(logName)

            elif action=='Pos':
                cmd_vals = env.userInput("Set desired position values (x,y,z): ",float)
                cmd_flag = env.userInput("Pos control On/Off (1,0): ",int)
                print()

                env.sendCmd(action,cmd_vals,cmd_flag)

            elif action=='Vel':
                cmd_vals = env.userInput("Set desired velocity values (x,y,z): ",float)
                cmd_flag = env.userInput("Vel control On/Off (1,0): ",int)
                print()

                env.sendCmd(action,cmd_vals,cmd_flag)

            elif action=='Yaw':
                yaw = env.userInput("Set desired yaw value: ",float)
                cmd_vals = [yaw,0,0]
                print()

                env.sendCmd(action,cmd_vals)

            elif action=='Tumble': # Turn on Tumble detection

                cmd_vals = [0,0,0]
                cmd_flag = env.userInput("Tumble Detection On/Off (1,0): ",int)
                print()

                env.sendCmd('Tumble',cmd_vals,cmd_flag)

            if action=='Stop': # Execute Ctrl_Reset or Stop action
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Rotors turned off\n")

                env.sendCmd(action,cmd_vals,cmd_flag)

            elif action=='dOmega':
                cmd_vals = env.userInput("Set desired dOmega values (x,y,z) rad/s^2: ",float)
                cmd_flag = env.userInput("dOmega control On/Off (1,0): ",int)
                env.sendCmd(action,cmd_vals,cmd_flag)


            elif action=='Load_Params': # Updates gain values from config file
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Reset ROS Parameters\n")

                env.setParams()
                env.sendCmd(action,cmd_vals,cmd_flag)

            elif action=='Policy':
                cmd_vals = env.userInput("Set desired (Tau,My_d) Policy: ",float)
                cmd_vals.append(0) # Append extra value to match framework
                cmd_flag = 1
                print()

                env.sendCmd(action,cmd_vals,cmd_flag)

            elif action=='Vel_traj':

                ## GET INPUT VALUES
                V_d,phi = env.userInput("Flight Velocity (V_d,phi):",float)

                ## DEFINE CARTESIAN VELOCITIES
                phi_rad = np.radians(phi)

                Vx_d = V_d*np.cos(phi_rad)
                Vz_d = V_d*np.sin(phi_rad)

                env.sendCmd('Vel_traj',cmd_vals=[env.pos[0],Vx_d,env.TrajAcc_Max[0]],cmd_flag=0)
                env.sendCmd('Vel_traj',cmd_vals=[env.pos[2],Vz_d,env.TrajAcc_Max[2]],cmd_flag=2)

            elif action=='Impact_traj':

                ## GET VEL CONDITIONS 
                V_d,phi = env.userInput("Flight Velocity (V_d,phi):",float)
                
                ## DEFINE CARTESIAN VELOCITIES
                phi_rad = np.radians(phi)

                Vx_d = V_d*np.cos(phi_rad)
                Vz_d = V_d*np.sin(phi_rad)

                x_impact = env.userInput("Desired impact position (x):",float)

                x_0,z_0 = env.VelTraj_StartPos(x_impact,[Vx_d,0,Vz_d])

                print(f"Desired start position x_0: {x_0:.2f} y_0: {0.0:.2f} z_0: {z_0:.2f}")
                str_input = env.userInput("Approve start position (y/n): ",str)

                if str_input == 'y':
                    env.sendCmd('P2P_traj',cmd_vals=[env.pos[0],x_0,env.TrajAcc_Max[0]],cmd_flag=0)
                    env.sendCmd('P2P_traj',cmd_vals=[env.pos[1],0.0,env.TrajAcc_Max[1]],cmd_flag=1)
                    env.sendCmd('P2P_traj',cmd_vals=[env.pos[2],z_0,env.TrajAcc_Max[2]],cmd_flag=2)

                    str_input = env.userInput("Approve flight (y/n): ",str)
                    if str_input == 'y':
                        env.sendCmd('Vel_traj',cmd_vals=[env.pos[0],Vx_d,env.TrajAcc_Max[0]],cmd_flag=0)
                        env.sendCmd('Vel_traj',cmd_vals=[env.pos[2],Vz_d,env.TrajAcc_Max[2]],cmd_flag=2)

                else:
                    print(f"Try again")


            elif action=='P2P_traj':
                ## GET INPUT VALUES
                x_d = env.userInput("Desired position (x,y,z):",float)
                env.sendCmd('P2P_traj',cmd_vals=[env.pos[0],x_d[0],env.TrajAcc_Max[0]],cmd_flag=0)
                env.sendCmd('P2P_traj',cmd_vals=[env.pos[1],x_d[1],env.TrajAcc_Max[1]],cmd_flag=1)
                env.sendCmd('P2P_traj',cmd_vals=[env.pos[2],x_d[2],env.TrajAcc_Max[2]],cmd_flag=2)


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


            elif action=='GZ_Plane_Pose':
                cmd_vals = env.userInput("Set desired position values (x,y,z): ",float)
                cmd_flag = env.userInput("Set desired plane angle [deg]: ",float)
                print()

                env._setPlanePose(cmd_vals,cmd_flag)

            else:
                print("Please try another command")

        except (ValueError, KeyError):
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