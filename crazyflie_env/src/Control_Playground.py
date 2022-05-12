#!/usr/bin/env python3
import threading,os
import rospy
import numpy as np



       
def cmd_send(env):
    while True:
        # Converts input number into action name
        cmd_dict = {
            0:'home',
            1:'pos',
            2:'vel',
            3:'acc',
            4:'tumble',
            5:'stop',
            6:'params',
            7:'moment',
            8:'policy',
            9:'vel_traj',
            10:'impact_traj',
            11:'sticky',
            13:'P2P_traj',
            19:'traj_tp',
            101:'reset',
            102:'cap_logging'
        }

        try:
            val = env.userInput("Cmd Type (0:home,1:pos,2:vel,5:stop,6:Param Reset,8:Policy,9:Traj,11:sticky,101:reset_state): ",int)
            print()
            action = cmd_dict[val]


            if action=='home': # Execute home or stop action
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Reset controller to default values\n")

                env.step('sticky',cmd_vals,0)
                env.step(action,cmd_vals,cmd_flag)

            elif action=='cap_logging':
                env.capLogging()

            elif action=='pos':
                cmd_vals = env.userInput("Set desired position values (x,y,z): ",float)
                cmd_flag = env.userInput("Pos control On/Off (1,0): ",int)
                print()

                env.step(action,cmd_vals,cmd_flag)

            elif action=='vel':
                cmd_vals = env.userInput("Set desired velocity values (x,y,z): ",float)
                cmd_flag = env.userInput("Vel control On/Off (1,0): ",int)
                print()

                env.step(action,cmd_vals,cmd_flag)

            elif action=='tumble': # Turn on tumble detection

                cmd_vals = [0,0,0]
                cmd_flag = env.userInput("Tumble Detection On/Off (1,0): ",int)
                print()

                env.step('tumble',cmd_vals,cmd_flag)

            if action=='stop': # Execute home or stop action
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Rotors turned of\n")

                env.step(action,cmd_vals,cmd_flag)


            elif action=='params': # Updates gain values from config file
                cmd_vals = [0,0,0]
                cmd_flag = 1
                print("Reset ROS Parameters\n")

                env.setParams()
                env.step(action,cmd_vals,cmd_flag)

            elif action=='moment':
                cmd_vals = env.userInput("Set desired moment values (x,y,z): ",float)
                cmd_flag = 1
                print()

                env.step(action,cmd_vals,cmd_flag)

            elif action=='policy':
                cmd_vals = env.userInput("Set desired (Tau,My_d) policy: ",float)
                cmd_vals.append(0) # Append extra value to match framework
                cmd_flag = 1
                print()

                env.step(action,cmd_vals,cmd_flag)

            elif action=='vel_traj':

                ## GET INPUT VALUES
                V_d,phi = env.userInput("Flight Velocity (V_d,phi):",float)

                ## DEFINE CARTESIAN VELOCITIES
                phi_rad = np.radians(phi)

                Vx_d = V_d*np.cos(phi_rad)
                Vz_d = V_d*np.sin(phi_rad)

                env.step('vel_traj',cmd_vals=[env.posCF[0],Vx_d,env.accCF_max[0]],cmd_flag=0)
                env.step('vel_traj',cmd_vals=[env.posCF[2],Vz_d,env.accCF_max[2]],cmd_flag=2)

            elif action=='impact_traj':

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
                    env.step('P2P_traj',cmd_vals=[env.posCF[0],x_0,env.accCF_max[0]],cmd_flag=0)
                    env.step('P2P_traj',cmd_vals=[env.posCF[1],y_0,env.accCF_max[1]],cmd_flag=1)
                    env.step('P2P_traj',cmd_vals=[env.posCF[2],z_0,env.accCF_max[2]],cmd_flag=2)

                    str_input = env.userInput("Approve flight (y/n): ",str)
                    if str_input == 'y':
                        env.step('vel_traj',cmd_vals=[x_0,Vx_d,env.accCF_max[0]],cmd_flag=0)
                        env.step('vel_traj',cmd_vals=[z_0,Vz_d,env.accCF_max[2]],cmd_flag=2)

                else:
                    print(f"Try again")




            elif action=='sticky':
                cmd_vals = [0,0,0]
                cmd_flag = env.userInput("Turn sticky pads On/Off (1,0): ",int)
                print()

                env.step(action,cmd_vals,cmd_flag)

            elif action=='P2P_traj':
                ## GET INPUT VALUES
                x_d = env.userInput("Desired position (x,y,z):",float)
                env.step('P2P_traj',cmd_vals=[env.posCF[0],x_d[0],env.accCF_max[0]],cmd_flag=0)
                env.step('P2P_traj',cmd_vals=[env.posCF[1],x_d[1],env.accCF_max[1]],cmd_flag=1)
                env.step('P2P_traj',cmd_vals=[env.posCF[2],x_d[2],env.accCF_max[2]],cmd_flag=2)


            elif action=='traj_tp':


                ## GET INPUT VALUES
                V_d,phi,alpha = env.userInput("Flight Velocity (V_d,phi,alpha):",float)

                ## DEFINE CARTESIAN VELOCITIES
                phi_rad = np.radians(phi)
                alpha_rad = np.radians(alpha)
                Vx_d = V_d*np.cos(phi_rad)*np.cos(alpha_rad)
                Vy_d = V_d*np.cos(phi_rad)*np.sin(alpha_rad)
                Vz_d = V_d*np.sin(phi_rad)

                ## ESTIMATE IMPACT POINT
                P_impact = env.impactEstimate(env.posCF,[Vx_d,Vy_d,Vz_d])

                ## CHECK VALID IMPACT POINT AND EXECUTE TRAJECTORY VIA SET_MODEL_STATE
                validate = input(f"Approve impact point (y/n): {P_impact[0]:.2f}, {P_impact[1]:.2f}, {P_impact[2]:.2f}\n")
                if validate == 'y':
                    env.traj_launch(env.posCF,[Vx_d,Vy_d,Vz_d])
                else:
                    pass



                    
            elif action == 'reset':
                print("Reset Pos/Vel -- Sticky off -- Controller Reset\n")
                env.reset_pos()

        except ValueError:
            print('\033[93m' + "INVALID INPUT: Try again" + '\x1b[0m')
            continue


if __name__ == '__main__':

    from RL_agents.rl_EM import rlEM_PEPGAgent
    from Crazyflie_env import CrazyflieEnv
    
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    agent = rlEM_PEPGAgent(n_rollouts=env.n_rollouts)

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    env.agent_name = agent.agent_type
    env.trial_name = f"Control_Playground--trial_{int(trial_num):02d}--{env.modelInitials()}"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"

    env.accCF_max = [1.0, 1.0, 3.1]

    env.createCSV(env.filepath)
    env.startLogging()

    # time.sleep(5)
    cmd_thread = threading.Thread(target=cmd_send,args=(env,))
    cmd_thread.start()   


    rospy.spin()