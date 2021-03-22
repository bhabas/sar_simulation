#!/usr/bin/env python3

import numpy as np
import time,os
from scipy.spatial.transform import Rotation



from Crazyflie_env import CrazyflieEnv
from rl_syspepg import rlsysPEPGAgent_reactive

os.system("clear")
np.set_printoptions(precision=2, suppress=True)



# ============================
##     Sim Initialization 
# ============================

## INIT GAZEBO ENVIRONMENT
env = CrazyflieEnv()
env.reset_pos() # Reset Gazebo pos
# env.launch_dashboard()
print("Environment done")








def runTrial():
    reset_vals = True
    
    # ============================
    ##          Episode         
    # ============================
    for k_ep in range(0,500):
        env.k_ep = k_ep

        ## CONVERT AGENT ARRAYS TO LISTS FOR PUBLISHING
        env.mu = agent.mu.flatten().tolist()
        env.sigma = agent.sigma.flatten().tolist()
        env.alpha_mu = agent.alpha_mu.flatten().tolist()
        env.alpha_sigma = agent.alpha_sigma.flatten().tolist()




        ## PREALLOCATE REWARD VEC AND OBTAIN THETA VALS
        reward = np.zeros(shape=(env.n_rollouts,1))


        print("=============================================")
        print("STARTING Episode # %d" %k_ep)
        print("=============================================")

        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )

        



        # ============================
        ##          Run 
        # ============================
        k_run = 0 # Reset run counter each episode
        while k_run < env.n_rollouts:
            env.k_run = k_run


            ## RESET TO INITIAL STATE
            env.step('home',ctrl_flag=1) # Reset control vals and functionality to default vals
            time.sleep(1.0) # Time for CF to settle

            if reset_vals == True:
        
                while True:
                    try:
                        ## Mu input:
                        mu_str = input("Input mu values: ")
                        num = list(map(float, mu_str.split()))
                        mu = np.asarray(num)

                        if len(mu) != 3:
                            raise Exception()
                        break
                    except:
                        print("Error: Enter mu_1, mu_2 and mu_3")

                while True:
                    try:
                        
                        v_str = input("Input V_ini (vx, vy, vz): ")
                        num = list(map(float, v_str.split()))
                        v_d = np.asarray(num)
                        vx_d,vy_d,vz_d = v_d
                        if len(v_d) != 3:
                            raise Exception()
                        break
                    except:
                        print("Error: Enter vz,vx,vy")



            
            


            ## INITIALIZE RUN PARAMETERS
            RREV_trigger = mu[0] # FoV expansion velocity [rad/s]
            G1 = mu[1]
            G2 = mu[2]
            env.policy = [RREV_trigger,G1,G2]
            env.step('policy',env.policy,ctrl_flag=1) # Arm controller policy
            
            env.vel_d = [vx_d,vy_d,vz_d] # [m/s]


            ## INIT RUN FLAGS
            t_step = 0
            z_max = 0 # [m]
            
            env.runComplete_flag = False
            env.flip_flag = False

            start_time_rollout = env.getTime()
            start_time_pitch = None
            state_history = None
            repeat_run= False
            error_str = ""

            ## PRINT RUN CONDITIONS AND POLICY
            print("\n!-------------------Episode # %d run # %d-----------------!" %(k_ep,k_run))
            print("RREV: %.3f \t Gain_1: %.3f \t Gain_2: %.3f \t Gain_3: %.3f" %(RREV_trigger, G1, G2, 0))
            print("Vx_d: %.3f \t Vy_d: %.3f \t Vz_d: %.3f" %(vx_d, vy_d, vz_d))


            # ============================
            ##          Rollout 
            # ============================
            env.step('pos',ctrl_flag=0) # Turn off pos control
            env.step('vel',env.vel_d,ctrl_flag=1) # Set desired vel
            env.step('sticky',ctrl_flag=1) # Enable sticky
            env.launch_IC(vx_d,vz_d)
 
            
            
            
            while True:
                
                ## DEFINE CURRENT STATE
                state = np.array(env.state_current)
                
                position = state[1:4] # [x,y,z]
                orientation_q = state[4:8] # Orientation in quat format
                vel = state[8:11] # [vx,vy,vz]
                vx,vy,vz = vel
                omega = state[11:14] # [wx,wy,wz]
                d = env.h_ceiling - position[2] # Vertical distance of drone from ceiling

                ## ORIENTATION DATA FROM STATE QUATERNION
                qw,qx,qy,qz = orientation_q
                R = Rotation.from_quat([qx,qy,qz,qw])
                R = R.as_matrix() # [b1,b2,b3] Body vectors
                RREV,OF_x,OF_y = vz/d, -vy/d, -vx/d # OF_x,y are mock optical flow vals assuming no body rotation
                

                # ============================
                ##      Record Keeping  
                # ============================
                ## Keep record of state vector every 10 time steps
                state = state[:, np.newaxis] # Convert [13,] array to [13,1] array
                
                if state_history is None:
                    state_history = state 
                else:
                    if t_step%1==0: # Append state_history columns with current state vector 
                        state_history = np.append(state_history, state, axis=1)
                        env.RL_Publish()
                        env.createCSV_flag = False


                # ============================
                ##    Termination Criteria 
                # ============================

                # # If time since triggered pitch exceeds [0.7s]   
                # if env.flip_flag and ((env.getTime()-start_time_pitch) > (0.7)):
                #     # I don't like this error formatting, feel free to improve on
                #     error_1 = "Rollout Completed: Pitch Timeout"
                #     error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_pitch,(env.getTime()-start_time_pitch))
                #     print(error_1 + "\n" + error_2)

                #     error_str = error_1 + error_2
                #     env.runComplete_flag = True

                # If position falls below max achieved height 
                z_max = max(position[2],z_max)
                if position[2] <= 0.85*z_max:
                    error_1 = "Rollout Completed: Falling Drone"
                    print(error_1)

                    error_str  = error_1
                    env.runComplete_flag = True

                # If time since run start exceeds [2.5s]
                if (env.getTime() - start_time_rollout) > (3.0):
                    error_1 = "Rollout Completed: Time Exceeded"
                    error_2 = "Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_rollout,(env.getTime()-start_time_rollout))
                    print(error_1 + "\n" + error_2)

                    error_str = error_1 + error_2
                    env.runComplete_flag = True

        
                    
                

                # ============================
                ##          Errors  
                # ============================

                ## If nan is found in state vector repeat sim run
                if any(np.isnan(state)): # gazebo sim becomes unstable, relaunch simulation
                    print("NAN found in state vector")
                    error_str = "Error: NAN found in state vector"
                    repeat_run = True
                    break




                # ============================
                ##       Run Completion  
                # ============================
                if env.runComplete_flag==True:

                    env.step('stop')
                    env.reset_pos()
                    ## There is a weird delay where it sometime won't publish ctrl_cmds until the next command is executed
                    ## I have no idea what's going on there but it may or may not have an effect?
                    ## I've got no idea...
                   
                    
                    
                    # reward[k_run] = agent.calcReward_pureLanding(state_history,env.h_ceiling)
                    reward[k_run] = 50
                    env.reward = reward[k_run]
                    print("Reward = %.3f" %(reward[k_run]))
                    print("!------------------------End Run------------------------! \n")                    
                    break

                t_step += 1

            
            ## =======  RUN COMPLETED  ======= ##
            if repeat_run == True:
                env.relaunch_sim()
            
            else:
                ## PUBLISH UPDATED REWARD VARIABLES

                env.reward = reward[k_run,0]
                env.RL_Publish()
                k_run += 1 # Move on to next run

                str = input("Input: \n(1): To play again \n(2): To repeat scenario \n(3): Game over :( \n")
                if str == '1':
                    k_run += 1
                    reset_vals = True

                elif str == '2':
                    k_run += 1
                    reset_vals = False
                else: 
                    break

            
        ## =======  EPISODE COMPLETED  ======= ##
        print("Episode # %d training, average reward %.3f" %(k_ep, np.mean(reward)))
        
       
    ## =======  MAX TRIALS COMPLETED  ======= ##


if __name__ == '__main__':


    ## SIM PARAMETERS
    env.n_rollouts = 10
    env.gamma = 0.95
    env.logging_flag = True
    env.h_ceiling = 3.0 # [m]




    ## LEARNING AGENTS
    agent = rlsysPEPGAgent_reactive(np.asarray(0),np.asarray(0),np.asarray(0),np.asarray(0))



    
    ## INITIAL CONDITIONS
    env.agent_name = "EM_PEPG"
    
    while True:
        start_time = time.strftime('_%Y-%m-%d_%H:%M:%S', time.localtime(time.time()))
        env.trial_name = f"RL_Policy_Playground-{start_time}"

        ## SEND MSG WHICH INCLUDES VARIABLE TO START NEW CSV
        env.createCSV_flag = True
        env.RL_Publish()

        ## RUN TRIAL AND RESTART GAZEBO FOR NEXT TRIAL RUN (NOT NECESSARY BUT MIGHT BE A GOOD IDEA?)
        runTrial()




