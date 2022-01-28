#!/usr/bin/env python3
import numpy as np
import os
import rospy


from Crazyflie_env import CrazyflieEnv
from RL_agents.rl_EM import rlEM_PEPGAgent
from ExecuteFlight import executeFlight


os.system("clear")
np.set_printoptions(precision=2, suppress=True)


if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    agent = rlEM_PEPGAgent()

    ## CHECK THAT POLICY IS IN RL MODE
    if rospy.get_param('/POLICY_TYPE') == 1:
        raise Exception("Policy is set to NN, change policy setting in Sim_Settings.yaml")


    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    env.agent_name = agent.agent_type
    env.trial_name = f"Policy_Playground--trial_{int(trial_num):02d}--{env.modelInitials}"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
    env.logging_flag = True
    env.create_csv(env.filepath)


    while True:
        try:
            V_d = float(input("Enter flight velocity: "))
        except:
            continue

        try:
            phi = float(input("Enter flight angle: "))
            phi_rad = np.radians(phi)
        except:
            continue

        try:
            policy = input("Enter 2-term policy (RREV,My): ")
            policy = [float(i) for i in policy.split(',')]
        except:
            continue

        env.vel_trial = [V_d*np.cos(phi_rad), 0.0, V_d*np.sin(phi_rad)] # [m/s]
        env.policy = [policy[0],policy[1],0]

    
        executeFlight(env,agent)
        env.k_ep += 1