#!/usr/bin/env python3
import threading,os
import rospy
import numpy as np

from RL_agents.rl_EM import rlEM_PEPGAgent
from Crazyflie_env import CrazyflieEnv

       
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
            9:'traj',
            11:'sticky',
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

                os.system("roslaunch crazyflie_launch params.launch")
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

            elif action=='traj':

                ## GET INPUT VALUES
                V_d,phi,alpha = env.userInput("Flight Velocity (V_d,phi,alpha):",float)

                ## DEFINE CARTESIAN VELOCITIES
                phi_rad = np.radians(phi)
                alpha_rad = np.radians(alpha)
                Vx_d = V_d*np.cos(phi_rad)*np.cos(alpha_rad)
                Vy_d = V_d*np.cos(phi_rad)*np.sin(alpha_rad)
                Vz_d = V_d*np.sin(phi_rad)

                ## ESTIMATE IMPACT POINT
                P_impact = env.impactEstimate(env.posCF_0,[Vx_d,Vy_d,Vz_d])

                ## CHECK VALID IMPACT POINT AND EXECUTE TRAJECTORY
                validate = input(f"Approve impact point (y/n): {P_impact[0]:.2f}, {P_impact[1]:.2f}, {P_impact[2]:.2f}\n")
                if validate == 'y':
                    env.step('traj',cmd_vals=[env.posCF_0[0],Vx_d,env.accCF_max[0]],cmd_flag=0)
                    env.step('traj',cmd_vals=[env.posCF_0[1],Vy_d,env.accCF_max[1]],cmd_flag=1)
                    env.step('traj',cmd_vals=[env.posCF_0[2],Vz_d,env.accCF_max[2]],cmd_flag=2)
                else:
                    pass

            elif action=='sticky':
                cmd_vals = [0,0,0]
                cmd_flag = env.userInput("Turn sticky pads On/Off (1,0): ",int)
                print()

                env.step(action,cmd_vals,cmd_flag)

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

        except:
            print('\033[93m' + "INVALID INPUT: Try again" + '\x1b[0m')
            continue


if __name__ == '__main__':
    
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    agent = rlEM_PEPGAgent(n_rollouts=env.n_rollouts)

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    env.agent_name = agent.agent_type
    env.trial_name = f"Control_Playground--trial_{int(trial_num):02d}--{env.modelInitials()}"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"

    env.createCSV(env.filepath)
    env.startLogging()

    # time.sleep(5)
    cmd_thread = threading.Thread(target=cmd_send,args=(env,))
    cmd_thread.start()   


    rospy.spin()