#!/usr/bin/env python3

import numpy as np
import time
import os
import sys
import rospy
import rospkg


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(0,BASE_PATH)

from crazyflie_msgs.msg import ImpactData,CtrlData
from crazyflie_rl.src.Crazyflie_env import CrazyflieEnv
from rospy.exceptions import ROSException

os.system("clear")
np.set_printoptions(precision=2, suppress=True)

def runTrial(env,vel_arr,phi_arr):

    env.create_csv(env.filepath)

    ## SEND MESSAGE FOR ALL NODES TO RESET TO DEFAULT VALUES
    env.reset_flag = True
    env.trialComplete_flag = False
    env.RL_Publish() # Publish updated flags

    for k_ep in range(0,len(vel_arr)):

        ## INIT LAUNCH/FLIGHT CONDITIONS
        phi_rad = phi_arr[k_ep]*np.pi/180
        vel = vel_arr[k_ep]
        vy_d = 0 # [m/s]
        env.vel_trial = [
            vel*np.cos(phi_rad), 
            vy_d, 
            vel*np.sin(phi_rad)] # [m/s]

        ## PRINT FLIGHT DATA
        print("=============================================")
        print("STARTING Episode # %d" %k_ep)
        print("=============================================")
        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
        
        try: # Use try block to catch raised exceptions and attempt rollout again

            ## IF CONTROLLER FAILS, RELAUNCH IT
            try:
                rospy.wait_for_message("/ctrl_data",CtrlData,timeout=5.0)
            except rospy.exceptions.ROSException:
                env.launch_controller()
                time.sleep(2)
                env.reset_pos()
                continue
                

            
        
            ## RESET TO INITIAL STATE
            env.step('home') # Reset control vals and functionality to default vals
            time.sleep(0.65) # Time for CF to settle [Real-Time seconds]

            



            env.step('policy',ctrl_vals=[0,0,0],ctrl_flag=1) # Arm controller policy


            ## RESET/UPDATE RUN CONDITIONS
            repeat_run= False

            start_time_rollout = env.getTime()
            start_time_pitch = np.nan
            start_time_impact = np.nan

            ## RESET LOGGING CONDITIONS 
            
            
            t_step = 0
            t_prev = 0.0


            

            
            onceFlag = False # Ensures flip data recorded only once (Needs a better name)
            onceFlag2 = False # Ensures impact data recorded only once (Needs a better name)

            
            



            ## PRINT RUN CONDITIONS AND POLICY
            print(f"\n!------------------- Episode # {k_ep:d} -----------------!")
            print(f"Vx_d: {env.vel_trial[0]:.3f} \t Vy_d: {env.vel_trial[1]:.3f} \t Vz_d: {env.vel_trial[2]:.3f}")
            print("\n")


            

            # ============================
            ##          Rollout 
            # ============================

            try:
                rospy.wait_for_message('/ctrl_data',CtrlData,timeout=2.0) # Wait to receive ctrl pub to run before continuing
            except ROSException:
                print("No ctrl message received")
                repeat_run = True
                
            ## VELOCITY IMPARTED FLIGHT
            pos_z = 0.4
            env.step('pos',ctrl_flag=0)                     # Turn off pos control
            env.step('vel',env.vel_trial,ctrl_flag=1)           # Set desired vel
            env.launch_IC(pos_z,env.vel_trial[0]+0.03,env.vel_trial[2])   # Use Gazebo to impart desired vel with extra vx to ensure -OF_y when around zero
            env.step('sticky',ctrl_flag=1)                  # Enable sticky pads
            
            # ## TRAJECTORY BASED FLIGHT
            # az_max = 3.0 # Max vertical accel
            # ax_max = 1.0 # Max horizontal accel (Made up)
            # env.step('traj',ctrl_vals=[0.4,env.vel_trial[2],az_max],ctrl_flag=2)
            # env.step('traj',ctrl_vals=[0.0,env.vel_trial[0],ax_max],ctrl_flag=0)

            
            
            while 1: # NOTE: [while 1:] is faster than [while True:]
                
                
                
                ## DEFINE CURRENT STATE
                state = env.state_current   # Collect state values here so they are thread-safe
                FM = np.array(env.FM)       # Motor thrust and Moments
                
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
                    print(f"RREV_tr={env.RREV_tr:.3f}, OF_y_tr={env.OF_y_tr:.3f}, My_d={My_d:.3f} [N*mm]")  
                    print(f"Pitch Time: {env.t_flip:.3f} [s]")
                    
                    onceFlag = True # Turns on to make sure this only runs once per rollout

                
                if ((env.impact_flag or env.body_contact) and onceFlag2 == False):
                    start_time_impact = env.getTime()
                    onceFlag2 = True


                # ============================
                ##      Record Keeping  
                # ============================

                # Check if sample of recorded data changed, if so then append csv (Reduces repeated data rows)
                if env.t != t_prev:
                # if t_step%1==0: 
                    # env.RL_Publish()
                    env.append_csv()

                    


                # ============================
                ##    Termination Criteria 
                # ============================
                if (env.impact_flag or env.body_contact) and ((env.getTime()-start_time_impact) > 1.0):
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

                    
                    
                    # print("\n")
                    # # env.RL_Publish() # Publish that rollout completed 
                    # # rospy.wait_for_message('/ceiling_force_sensor',ImpactData)

                    # # reward_arr[k_run] = agent.calcReward_pureLanding(env)
                    # reward_arr[k_run] = agent.calcReward_Impact(env)
                    # env.reward = reward_arr[k_run,0]
                    # env.reward_avg = reward_arr[np.nonzero(reward_arr)].mean()
                    # env.reward_inputs = [env.z_max,env.pitch_sum,env.pitch_max]
                    
                    
                    # print(f"Reward = {env.reward:.3f}")
                    # # print(f"# of Leg contacts: {sum(env.pad_contacts)}")
                    # print("!------------------------End Run------------------------! \n")  

                    

                    ## RUN DATA LOGGING
                    env.append_csv_blank()
                    env.append_IC()
                    env.append_flip()
                    env.append_impact()
                    env.append_csv_blank()

                    


                    ## PUBLISH RL DATA AND RESET LOGS/POSITIONS FOR NEXT ROLLOUT
                    env.reset_flag = True
                    env.RL_Publish() # Publish that rollout completed 
                    
                    env.reset_pos()

                    ## RESET/UPDATE RUN CONDITIONS
                    env.runComplete_flag = False
                    env.reset_flag = False
                    env.error_str = ""
                    env.clear_rollout_Data()
                
                    break # Break from run loop
                    
                t_step += 1  
                t_prev = env.t   
            
            ## =======  RUN COMPLETED  ======= ##
            if repeat_run == True: # Runs when error detected
                env.relaunch_sim()
            
            else:
                ## PUBLISH UPDATED REWARD VARIABLES
                # env.reward = reward_arr[k_run,0]
                # env.reward_avg = reward_arr[np.nonzero(reward_arr)].mean()
                # env.RL_Publish()
                # k_run += 1 # When succesful move on to next run
                pass 
        except rospy.service.ServiceException: ## IF SIM EXCEPTION RAISED THEN CONTINUE BACK TO TRY BLOCK UNTIL SUCCESSFUL COMPLETION
            continue

if __name__ == '__main__':
    ## DEFINE FLIGHT CONDITIONS
    num_flights = 20
    vels = np.random.uniform(2.5,3.5,num_flights)
    angles = np.random.uniform(40,90,num_flights)
    print(vels,angles)


    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)

    ## INIT LOGGING DATA
    trial_num = 24
    env.trial_name = f"NN_Policy--trial_{int(trial_num):02d}--Model_{env.modelName[10:]}"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
    env.logging_flag = True

    ## RUN TRIAL
    env.RL_Publish() # Publish data to rl_data topic
    time.sleep(3)

    runTrial(env,vels,angles)