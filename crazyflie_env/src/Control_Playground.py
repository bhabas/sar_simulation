#!/usr/bin/env python3
import threading,os
import rospy
import numpy as np



       
def cmd_send(env):
    while True:
        # Converts input number into action name
        cmd_dict = {
            0:'Ctrl_Reset',
            1:'Pos',
            2:'Vel',
            5:'Stop',
            7:'Moment',
            8:'Policy',

            10:'P2P_traj',
            11:'Vel_traj',
            12:'Impact_traj',

            20:'Tumble',
            21:'Load_Params',
            22:'Cap_Logging',

            90:'GZ_traj',
            91:'GZ_reset',
            92:'StickyPads',
        }

        try:
            print("========== Command Types ==========")
            print("0:Ctrl_Reset, \t1:Pos, \t\t2:Vel, \t\t5:Stop, \t8:Policy")
            print("10:P2P_traj, \t11:Vel_traj, \t12:Impact_traj,")
            print("20:Tumble, \t21:Load_Params, 22:Cap_Logging,")
            print("90:GZ_traj, \t91:GZ_reset, \t92:StickyPads")
            val = env.userInput("\nCmd: ",int)
            print()
            action = cmd_dict[val]


            if action=='Ctrl_Reset': # Execute Ctrl_Reset or Stop action
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Reset controller to default values\n")

                env.SendCmd('StickyPads',cmd_vals,0)
                env.SendCmd(action,cmd_vals,cmd_flag)

            elif action=='Cap_Logging':
                env.capLogging(logName)

            elif action=='Pos':
                cmd_vals = env.userInput("Set desired position values (x,y,z): ",float)
                cmd_flag = env.userInput("Pos control On/Off (1,0): ",int)
                print()

                env.SendCmd(action,cmd_vals,cmd_flag)

            elif action=='Vel':
                cmd_vals = env.userInput("Set desired velocity values (x,y,z): ",float)
                cmd_flag = env.userInput("Vel control On/Off (1,0): ",int)
                print()

                env.SendCmd(action,cmd_vals,cmd_flag)

            elif action=='Tumble': # Turn on Tumble detection

                cmd_vals = [0,0,0]
                cmd_flag = env.userInput("Tumble Detection On/Off (1,0): ",int)
                print()

                env.SendCmd('Tumble',cmd_vals,cmd_flag)

            if action=='Stop': # Execute Ctrl_Reset or Stop action
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Rotors turned off\n")

                env.SendCmd(action,cmd_vals,cmd_flag)

            elif action=='Moment':
                cmd_vals = env.userInput("Set desired Moment values (x,y,z) N*mm: ",float)
                cmd_flag = env.userInput("Moment control On/Off (1,0): ",int)
                env.SendCmd(action,cmd_vals,cmd_flag)


            elif action=='Load_Params': # Updates gain values from config file
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Reset ROS Parameters\n")

                env.setParams()
                env.SendCmd(action,cmd_vals,cmd_flag)

            elif action=='moment':
                cmd_vals = env.userInput("Set desired moment values (x,y,z): ",float)
                cmd_flag = 1
                print()

                env.SendCmd(action,cmd_vals,cmd_flag)

            elif action=='Policy':
                cmd_vals = env.userInput("Set desired (Tau,My_d) Policy: ",float)
                cmd_vals.append(0) # Append extra value to match framework
                cmd_flag = 1
                print()

                env.SendCmd(action,cmd_vals,cmd_flag)

            elif action=='Vel_traj':

                ## GET INPUT VALUES
                V_d,phi = env.userInput("Flight Velocity (V_d,phi):",float)

                ## DEFINE CARTESIAN VELOCITIES
                phi_rad = np.radians(phi)

                Vx_d = V_d*np.cos(phi_rad)
                Vz_d = V_d*np.sin(phi_rad)

                env.SendCmd('Vel_traj',cmd_vals=[env.posCF[0],Vx_d,env.accCF_max[0]],cmd_flag=0)
                env.SendCmd('Vel_traj',cmd_vals=[env.posCF[2],Vz_d,env.accCF_max[2]],cmd_flag=2)

            elif action=='Impact_traj':

                ## GET VEL CONDITIONS 
                V_d,phi,d_vel = env.userInput("Flight Velocity (V_d,phi,d_vel):",float)
                
                ## DEFINE CARTESIAN VELOCITIES
                phi_rad = np.radians(phi)

                Vx_d = V_d*np.cos(phi_rad)
                Vz_d = V_d*np.sin(phi_rad)

                x_impact,y_0 = env.userInput("Desired impact position (x,y):",float)

                x_0,z_0 = env.VelTraj_StartPos(x_impact,[Vx_d,0,Vz_d],d_vel=d_vel)

                print(f"Desired start position x_0: {x_0:.2f} y_0: {y_0:.2f} z_0: {z_0:.2f}")
                str_input = env.userInput("Approve start position (y/n): ",str)

                if str_input == 'y':
                    env.SendCmd('P2P_traj',cmd_vals=[env.posCF[0],x_0,env.accCF_max[0]],cmd_flag=0)
                    env.SendCmd('P2P_traj',cmd_vals=[env.posCF[1],y_0,env.accCF_max[1]],cmd_flag=1)
                    env.SendCmd('P2P_traj',cmd_vals=[env.posCF[2],z_0,env.accCF_max[2]],cmd_flag=2)

                    str_input = env.userInput("Approve flight (y/n): ",str)
                    if str_input == 'y':
                        env.SendCmd('Vel_traj',cmd_vals=[env.posCF[0],Vx_d,env.accCF_max[0]],cmd_flag=0)
                        env.SendCmd('Vel_traj',cmd_vals=[env.posCF[2],Vz_d,env.accCF_max[2]],cmd_flag=2)

                else:
                    print(f"Try again")




            elif action=='StickyPads':
                cmd_vals = [0,0,0]
                cmd_flag = env.userInput("Turn sticky pads On/Off (1,0): ",int)
                print()

                env.SendCmd(action,cmd_vals,cmd_flag)

            elif action=='P2P_traj':
                ## GET INPUT VALUES
                x_d = env.userInput("Desired position (x,y,z):",float)
                env.SendCmd('P2P_traj',cmd_vals=[env.posCF[0],x_d[0],env.accCF_max[0]],cmd_flag=0)
                env.SendCmd('P2P_traj',cmd_vals=[env.posCF[1],x_d[1],env.accCF_max[1]],cmd_flag=1)
                env.SendCmd('P2P_traj',cmd_vals=[env.posCF[2],x_d[2],env.accCF_max[2]],cmd_flag=2)


            elif action=='GZ_traj':


                ## GET INPUT VALUES
                V_d,phi,alpha = env.userInput("Flight Velocity (V_d,phi,alpha):",float)

                ## DEFINE CARTESIAN VELOCITIES
                phi_rad = np.radians(phi)
                alpha_rad = np.radians(alpha)
                Vx_d = V_d*np.cos(phi_rad)*np.cos(alpha_rad)
                Vy_d = V_d*np.cos(phi_rad)*np.sin(alpha_rad)
                Vz_d = V_d*np.sin(phi_rad)

                ## ESTIMATE IMPACT POINT
                env.Vel_Launch(env.posCF,[Vx_d,Vy_d,Vz_d])
                
                    
            elif action == 'GZ_reset':
                print("Reset Pos/Vel -- Sticky off -- Controller Reset\n")
                env.reset_pos()

        except ValueError:
            print('\033[93m' + "INVALID INPUT: Try again" + '\x1b[0m')
            continue


if __name__ == '__main__':

    # from Crazyflie_env import CrazyflieEnv
    from Crazyflie_env import CrazyflieEnv
    
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    logName = f"Control_Playground--trial_{int(trial_num):02d}--{env.modelInitials()}.csv"


    env.accCF_max = [1.0, 1.0, 3.1]

    env.createCSV(logName)
    env.startLogging(logName)

    # time.sleep(5)
    cmd_thread = threading.Thread(target=cmd_send,args=(env,))
    cmd_thread.start()   


    rospy.spin()