#!/usr/bin/env python3
import numpy as np
import time
import os
import rospy
from rospy.exceptions import ROSException



from crazyflie_msgs.msg import CF_StateData
from rosgraph_msgs.msg import Clock

os.system("clear")
np.set_printoptions(precision=2, suppress=True)

def executeFlight(env,agent):

    # # MAKE SURE CONTROLLER IS WORKING
    # while True:
    #     try:
    #         rospy.wait_for_message("/clock",Clock,timeout=20)
    #         rospy.wait_for_message("/CF_DC/StateData",CF_StateData,timeout=5.0)
    #         break

    #     except rospy.exceptions.ROSException:
    #         print("Restarting Controller")
    #         env.launch_controller()
    #         time.sleep(2)
    #         env.reset_pos()
    #         continue

    ## RESET TO INITIAL STATE
    env.SendCmd("GZ_reset")
    env.SendCmd("Ctrl_Reset") # Reset control vals and functionality to default vals
    env.sleep(0.5) # Time for CF to settle [Real-Time seconds]
    env.startLogging()


    

    # ============================
    ##          Rollout 
    # ============================
    env.SendCmd('StickyPads',cmd_flag=1)              # Enable sticky pads

    tau_0 = 0.6
    z_0 = env.h_ceiling - tau_0*env.vel_d[2]
    env.Vel_Launch([0,0,z_0],env.vel_d)
    env.sleep(0.1)
    env.SendCmd("Policy",env.policy,cmd_flag=1) # Arm policy inside controller

    ## RESET/UPDATE RUN CONDITIONS
    start_time_rollout = env.getTime()
    start_time_pitch = np.nan
    start_time_impact = np.nan

    ## RESET LOGGING CONDITIONS 
    onceFlag_flip = False    # Ensures flip data recorded only once
    onceFlag_impact = False   # Ensures impact data recorded only once 

    ## PRINT RUN CONDITIONS AND POLICY
    print(f"Vx_d: {env.vel_d[0]:.3f} \t Vy_d: {env.vel_d[1]:.3f} \t Vz_d: {env.vel_d[2]:.3f}")
    print("\n")



    while True: 


        # ============================
        ##      Pitch Recording 
        # ============================

        if (env.flip_flag == True and onceFlag_flip == False):
            start_time_pitch = env.getTime() # Starts countdown for when to reset run

            # Recieve flip moments from controller and then update class var to be sent out of /rl_data topic
            Mx_d = env.FM_tr[1] 
            My_d = env.FM_tr[2]
            Mz_d = env.FM_tr[3]

        
            print("----- pitch starts -----")
            print(f"vx={env.velCF[0]:.3f}, vy={env.velCF[1]:.3f}, vz={env.velCF[2]:.3f}")
            print(f"Tau_tr={env.Tau_tr:.3f}, OFy_tr={env.OFy_tr:.3f}, My_d={My_d:.3f} [N*mm]")  
            print(f"Pitch Time: {env.t_tr:.3f} [s]")
            
            onceFlag_flip = True # Turns on to make sure this only runs once per rollout

        
        if ((env.impact_flag or env.BodyContact_flag) and onceFlag_impact == False):
            start_time_impact = env.getTime()
            onceFlag_impact = True

        # ============================
        ##    Termination Criteria 
        # ============================
        if (env.impact_flag or env.BodyContact_flag) and ((env.getTime()-start_time_impact) > 0.5):
            env.error_str = "Rollout Completed: Impact Timeout"
            print(env.error_str)

            env.runComplete_flag = True

        # IF TIME SINCE TRIGGERED PITCH EXCEEDS [1.5s]  
        elif env.flip_flag and ((env.getTime()-start_time_pitch) > (2.25)):
            env.error_str = "Rollout Completed: Pitch Timeout"
            print(env.error_str)

            env.runComplete_flag = True

        # IF POSITION FALLS BELOW FLOOR HEIGHT
        elif env.velCF[2] <= -0.5 and env.posCF[2] <= 1.5: # Note: there is a lag with this at high RTF
            env.error_str = "Rollout Completed: Falling Drone"
            print(env.error_str)

            env.runComplete_flag = True

        # IF TIME SINCE RUN START EXCEEDS 
        elif (env.getTime() - start_time_rollout) > (5.0):
            env.error_str = "Rollout Completed: Time Exceeded"
            print(env.error_str)

            env.runComplete_flag = True

        # ============================
        ##          Errors  
        # ============================

        ## IF NAN IS FOUND IN STATE VECTOR REPEAT RUN (Model collision Error)
        if any(np.isnan(env.velCF)): 
            env.error_str = "Error: NAN found in state vector"
            print(env.error_str)
            env.repeat_run = True
            break

        # ============================
        ##       Run Completion  
        # ============================
        if env.runComplete_flag==True:

            print("\n")
            env.reward = agent.calcReward_Impact(env)
            env.reward_inputs = [env.d_ceil_max,env.pitch_sum,env.pitch_max]
            
            
            print(f"Reward = {env.reward:.3f}")
            print(f"# of Leg contacts: {env.pad_connections}")
            print("!------------------------End Run------------------------! \n")  

            

            ## RUN DATA LOGGING
            env.RL_Publish()
            env.capLogging()

            


            ## PUBLISH RL DATA AND RESET LOGS/POSITIONS FOR NEXT ROLLOUT
            env.runComplete_flag = False            
            env.reset_pos()
            env.reset_reward_terms()
        
            break # Break from run loop
            

    ## =======  RUN COMPLETED  ======= ##
    


if __name__ == '__main__':

    from Crazyflie_env import CrazyflieEnv
    from RL_agents.rl_EM import rlEM_PEPGAgent

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    agent = rlEM_PEPGAgent(n_rollouts=env.n_rollouts)


    # ============================
    ##     LEARNING CONDITIONS  
    # ============================

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    env.trial_name = f"ExampleFlight--trial_{int(trial_num):02d}--{env.modelInitials()}"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
    env.Logging_Flag = True
    env.createCSV(env.filepath)

    V_d = 2.5
    phi = 90
    phi_rad = np.radians(phi)
    env.vel_d = [V_d*np.cos(phi_rad), 0.0, V_d*np.sin(phi_rad)] # [m/s]
    env.policy = [0.27,7.0,0] # NN policy


    ## RUN TRIAL

    while True:
        executeFlight(env,agent)