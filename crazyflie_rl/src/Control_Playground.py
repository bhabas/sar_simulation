#!/usr/bin/env python3
import threading,os
import rospy
import time

from RL_agents.rl_EM import rlEM_PEPGAgent
from Crazyflie_env import CrazyflieEnv
from ExecuteFlight import executeFlight

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState



os.system("clear")

       
def cmd_send(env):
    while True:
        # Converts input number into action name
        cmd_dict = {0:'home',
            1:'pos',
            2:'vel',
            3:'acc',
            4:'tumble',
            5:'stop',
            6:'gains',
            7:'moment',
            8:'policy',
            9:'traj',
            11:'sticky',
            101:'reset'}
        try:
            val = float(input("\nCmd Type (0:home,1:pos,2:vel,3:acc,4:omega,5:stop,101:reset): "))
        except:
            continue
        action = cmd_dict[val]

        if action=='home' or action == 'stop': # Execute home or stop action
            ctrl_vals = [0,0,0]
            ctrl_flag = 1
            env.step('sticky',ctrl_vals,0)
            env.step(action,ctrl_vals,ctrl_flag)

        elif action=='gains': # Updates gain values from config file
            
            try:
                ctrl_vals = [0,0,0]
                ctrl_flag = 1
                os.system("roslaunch crazyflie_launch params.launch")
                env.step(action,ctrl_vals,ctrl_flag)
            except:
                continue

            
            
        elif action == 'tumble': # Turn on tumble detection

            ctrl_vals = input("\nControl Vals (x,y,z): ")
            ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
            ctrl_flag = 1.0

            env.step('tumble',ctrl_vals,ctrl_flag)

        elif action == 'reset':
            env.reset_pos()


        else:
            ctrl_vals = input("\nControl Vals (x,y,z): ")
            ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
            ctrl_flag = float(input("\nController On/Off (1,0): "))
            env.step(action,ctrl_vals,ctrl_flag)

       
def logFlight(env):

    ## RESET LOGGING CONDITIONS 
    t_prev = 0.0

    while 1: 

        # If time changes then append csv file
        if env.t != t_prev:
            env.append_csv()

        t_prev = env.t   


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



    cmd_thread = threading.Thread(target=cmd_send,args=(env,))
    cmd_thread.start()   

    time.sleep(2)
    logging_thread = threading.Thread(target=logFlight,args=(env,))
    logging_thread.start()   

    rospy.spin()