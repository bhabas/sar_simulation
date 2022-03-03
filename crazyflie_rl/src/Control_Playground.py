#!/usr/bin/env python3
import threading,os
import rospy
import time

from RL_agents.rl_EM import rlEM_PEPGAgent
from Crazyflie_env import CrazyflieEnv
from ExecuteFlight import executeFlight

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from rosgraph_msgs.msg import Clock




os.system("clear")

       
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
            101:'reset',
        }

        try:
            val = env.userInput("Cmd Type (0:home,1:pos,2:vel,5:stop,6:Param Reset,8:Policy,9:Traj,11:sticky,101:reset_state): ",int)
            print()
            action = cmd_dict[val]


            if action=='home': # Execute home or stop action
                ctrl_vals = [0,0,0]
                ctrl_flag = 1
                print("Reset controller to default values\n")

                env.step('sticky',ctrl_vals,0)
                env.step(action,ctrl_vals,ctrl_flag)

            elif action=='pos':
                ctrl_vals = env.userInput("Set desired position values (x,y,z): ",float)
                ctrl_flag = env.userInput("Pos control On/Off (1,0): ",int)
                print()

                env.step(action,ctrl_vals,ctrl_flag)

            elif action=='vel':
                ctrl_vals = env.userInput("Set desired velocity values (x,y,z): ",float)
                ctrl_flag = env.userInput("Vel control On/Off (1,0): ",int)
                print()

                env.step(action,ctrl_vals,ctrl_flag)

            elif action=='tumble': # Turn on tumble detection

                ctrl_vals = [0,0,0]
                ctrl_flag = env.userInput("Tumble Detection On/Off (1,0): ",int)
                print()

                env.step('tumble',ctrl_vals,ctrl_flag)

            if action=='stop': # Execute home or stop action
                ctrl_vals = [0,0,0]
                ctrl_flag = 1
                print("Rotors turned of\n")

                env.step(action,ctrl_vals,ctrl_flag)


            elif action=='params': # Updates gain values from config file
                ctrl_vals = [0,0,0]
                ctrl_flag = 1
                print("Reset ROS Parameters\n")

                os.system("roslaunch crazyflie_launch params.launch")
                env.step(action,ctrl_vals,ctrl_flag)

            elif action=='moment':
                ctrl_vals = env.userInput("Set desired moment values (x,y,z): ",float)
                ctrl_flag = 1
                print()

                env.step(action,ctrl_vals,ctrl_flag)

            elif action=='policy':
                ctrl_vals = env.userInput("Set desired (RREV,My_d) policy: ",float)
                ctrl_vals.append(0) # Append extra value to match framework
                ctrl_flag = 1
                print()

                env.step(action,ctrl_vals,ctrl_flag)

            elif action=='traj':
                ctrl_vals = env.userInput("Set desired velocity trajectory (Pos_0,Vel_d,Acc_max): ",float)
                ctrl_flag = env.userInput("Set desired axis (x:0,y:1,z:2): ",int)
                print()

                env.step(action,ctrl_vals,ctrl_flag)

            elif action=='sticky':
                ctrl_vals = [0,0,0]
                ctrl_flag = env.userInput("Turn sticky pads On/Off (1,0): ",int)
                print()

                env.step(action,ctrl_vals,ctrl_flag)
            
            elif action == 'reset':
                print("Reset Pos/Vel -- Sticky off -- Controller Reset\n")
                env.reset_pos()

        except:
            print('\033[93m' + "INVALID INPUT: Try again" + '\x1b[0m')
            continue

       
def logFlight(env):

    ## RESET LOGGING CONDITIONS 
    t_prev = 0.0
    t_start = rospy.Time.now().to_sec()

    while env.t - t_start <= 30: 

        # If time changes then append csv file
        if env.t != t_prev:
            env.append_csv()

        t_prev = env.t   
    
    env.append_IC()
    env.append_flip()
    env.append_impact()
    env.append_csv_blank()


if __name__ == '__main__':
    
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    agent = rlEM_PEPGAgent(n_rollouts=env.n_rollouts)

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    env.agent_name = agent.agent_type
    env.trial_name = f"Control_Playground--trial_{int(trial_num):02d}--{env.modelInitials}"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
    env.logging_flag = True
    env.create_csv(env.filepath)

    time.sleep(5)
    cmd_thread = threading.Thread(target=cmd_send,args=(env,))
    cmd_thread.start()   


    logging_thread = threading.Thread(target=logFlight,args=(env,))
    logging_thread.start()   

    rospy.spin()