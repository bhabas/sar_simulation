#!/usr/bin/env python3
import numpy as np
import time,os
import rospy
from crazyflie_msgs.msg import ImpactData,CtrlData
import math


from nav_msgs.msg import Odometry
from Crazyflie_env import CrazyflieEnv
from RL_agents.rl_EM import rlEM_PEPGAgent
from rospy.exceptions import ROSException

os.system("clear")
np.set_printoptions(precision=2, suppress=True)

def executeFlight(env,agent):

    ## SEND MESSAGE FOR ALL NODES TO RESET TO DEFAULT VALUES
    env.reset_flag = True
    env.trialComplete_flag = False
    env.RL_Publish() # Publish that rollout completed 

    ## MAKE SURE CONTROLLER IS WORKING
    while True:
        try:
            rospy.wait_for_message('/env/global_state_data',Odometry) # Wait for global state message before resuming training
            rospy.wait_for_message("/ctrl_data",CtrlData,timeout=5.0)
            break

        except rospy.exceptions.ROSException:
            print("Restarting Controller")
            env.launch_controller()
            time.sleep(2)
            env.reset_pos()
            continue

    ## RESET TO INITIAL STATE
    env.step("home") # Reset control vals and functionality to default vals
    time.sleep(0.65) # Time for CF to settle [Real-Time seconds]

    env.step("policy",env.policy,ctrl_flag=1) # Arm policy inside controller

    ## RESET/UPDATE RUN CONDITIONS
    repeat_run= False

    start_time_rollout = env.getTime()
    start_time_pitch = np.nan
    start_time_impact = np.nan

    ## RESET LOGGING CONDITIONS 
    t_prev = 0.0

    onceFlag = False    # Ensures flip data recorded only once
    onceFlag2 = False   # Ensures impact data recorded only once 

    ## PRINT RUN CONDITIONS AND POLICY
    print(f"Vx_d: {env.vel_trial[0]:.3f} \t Vy_d: {env.vel_trial[1]:.3f} \t Vz_d: {env.vel_trial[2]:.3f}")
    print("\n")

    # ============================
    ##          Rollout 
    # ============================
    z_0 = 0.3986
    env.step('sticky',ctrl_flag=1)              # Enable sticky pads
    env.step('pos',ctrl_flag=0)                 # Turn off pos control
    env.step('vel',env.vel_trial,ctrl_flag=1)   # Set desired vel
    env.launch_IC(                              # Use Gazebo to impart desired vel with extra vx to ensure -OFy when around zero
        z_0,
        env.vel_trial[0]+0.03,
        env.vel_trial[2])   


    while 1: 

        ## DEFINE CURRENT STATE
        state = env.state_current   # Collect state values here so they are thread-safe
        
        vx,vy,vz = state[7:10] # [vx,vy,vz]

        # ============================
        ##      Pitch Recording 
        # ============================

        if (env.flip_flag == True and onceFlag == False):
            start_time_pitch = env.getTime() # Starts countdown for when to reset run

            # Recieve flip moments from controller and then update class var to be sent out of /rl_data topic
            Mx_d = env.FM_flip[1] 
            My_d = env.FM_flip[2]
            Mz_d = env.FM_flip[3]
            env.M_d = [Mx_d,My_d,Mz_d] # [N*mm]

        
            print("----- pitch starts -----")
            print(f"vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}")
            print(f"RREV_tr={env.RREV_tr:.3f}, OFy_tr={env.OFy_tr:.3f}, My_d={My_d:.3f} [N*mm]")  
            print(f"Pitch Time: {env.t_flip:.3f} [s]")
            
            onceFlag = True # Turns on to make sure this only runs once per rollout

        
        if ((env.impact_flag or env.body_contact) and onceFlag2 == False):
            start_time_impact = env.getTime()
            onceFlag2 = True

        # ============================
        ##      Record Keeping  
        # ============================

        # If time changes then append csv file
        if env.t != t_prev:
            env.append_csv()

        # ============================
        ##    Termination Criteria 
        # ============================
        if (env.impact_flag or env.body_contact) and ((env.getTime()-start_time_impact) > 0.5):
            env.error_str = "Rollout Completed: Impact Timeout"
            print(env.error_str)

            env.runComplete_flag = True

        # IF TIME SINCE TRIGGERED PITCH EXCEEDS [1.5s]  
        elif env.flip_flag and ((env.getTime()-start_time_pitch) > (2.25)):
            env.error_str = "Rollout Completed: Pitch Timeout"
            print(env.error_str)

            env.runComplete_flag = True

        # IF POSITION FALLS BELOW FLOOR HEIGHT
        elif env.position[2] <= -8.0: # Note: there is a lag with this at high RTF
            env.error_str = "Rollout Completed: Falling Drone"
            print(env.error_str)

            env.runComplete_flag = True

        # IF TIME SINCE RUN START EXCEEDS [6.0s]
        elif (env.getTime() - start_time_rollout) > (5.0):
            env.error_str = "Rollout Completed: Time Exceeded"
            print(env.error_str)

            env.runComplete_flag = True

        # ============================
        ##          Errors  
        # ============================

        ## IF NAN IS FOUND IN STATE VECTOR REPEAT RUN (Model collision Error)
        if any(np.isnan(env.state_current)): 
            env.error_str = "Error: NAN found in state vector"
            print(env.error_str)
            repeat_run = True
            break

        if np.abs(env.position[1]) >= 1.0: # If CF goes crazy it'll usually shoot out in y-direction
            env.error_str = "Error: Y-Position Exceeded"
            print(env.error_str)
            repeat_run = True
            break

        # ============================
        ##       Run Completion  
        # ============================
        if env.runComplete_flag==True:

            
            
            print("\n")
            env.reward = agent.calcReward_Impact(env)
            env.reward_inputs = [env.z_max,env.pitch_sum,env.pitch_max]
            
            
            print(f"Reward = {env.reward:.3f}")
            print(f"# of Leg contacts: {len(env.pad_contacts)}")
            print("!------------------------End Run------------------------! \n")  

            

            ## RUN DATA LOGGING
            env.append_csv_blank()
            env.append_IC()
            env.append_flip()
            env.append_impact()
            env.append_csv_blank()

            


            ## PUBLISH RL DATA AND RESET LOGS/POSITIONS FOR NEXT ROLLOUT
            env.reset_flag = True
            env.runComplete_flag = False
            env.RL_Publish() # Publish that rollout completed 
            
            env.reset_pos()
            env.clear_rollout_Data()
        
            break # Break from run loop
            
        t_prev = env.t   

    ## =======  RUN COMPLETED  ======= ##
    if repeat_run == True: # Runs when error detected
        env.relaunch_sim()
    
    else:
        ## PUBLISH UPDATED REWARD VARIABLES
        env.RL_Publish()


if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    agent = rlEM_PEPGAgent(n_rollouts=env.n_rollouts)


    # ============================
    ##     LEARNING CONDITIONS  
    # ============================

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    env.agent_name = agent.agent_type
    env.trial_name = f"ExampleFlight--trial_{int(trial_num):02d}--{env.modelInitials}"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
    env.logging_flag = True
    env.create_csv(env.filepath)

    V_d = 2.5
    phi = 60
    phi_rad = np.radians(phi)
    env.vel_trial = [V_d*np.cos(phi_rad), 0.0, V_d*np.sin(phi_rad)] # [m/s]
    env.policy = [5,8.0,0] # NN policy


    ## RUN TRIAL
    env.RL_Publish() # Publish data to rl_data topic
    # time.sleep(3)

    while True:
        executeFlight(env,agent)