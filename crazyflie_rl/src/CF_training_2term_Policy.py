#!/usr/bin/env python3


import numpy as np
import time,os
import rospy
from crazyflie_msgs.msg import CtrlData



from Crazyflie_env import CrazyflieEnv
# from rl_syspepg import rlsysPEPGAgent_reactive,rlsysPEPGAgent_adaptive
from RL_agents.rl_EM import rlEM_PEPGAgent
# from rl_cma import CMA_basic,CMA,CMA_sym
from rospy.exceptions import ROSException

os.system("clear")
np.set_printoptions(precision=2, suppress=True)


def runTraining(env,agent,V_d,phi,k_epMax=250):
    env.create_csv(env.filepath)

    ## INIT LAUNCH/FLIGHT CONDITIONS
    phi_rad = phi*np.pi/180
    vy_d = 0 # [m/s]
    env.vel_d = [V_d*np.cos(phi_rad), vy_d, V_d*np.sin(phi_rad)] # [m/s]
    
    # ============================
    ##          Episode         
    # ============================
    
    for k_ep in range(0,k_epMax):

        ## UPDATE EPISODE NUMBER
        env.k_ep = k_ep

        ## CONVERT AGENT ARRAYS TO LISTS FOR PUBLISHING
        env.mu = agent.mu.flatten().tolist()                    # Mean for Gaussian distribution
        env.sigma = agent.sigma.flatten().tolist()              # Standard Deviation for Gaussian distribution

        env.alpha_mu = agent.alpha_mu.flatten().tolist()        # Learning rate for mu (PEPG)
        env.alpha_sigma = agent.alpha_sigma.flatten().tolist()  # Learning rate for sigma (PEPG)

        
        ## PREALLOCATE REWARD VEC AND OBTAIN THETA VALS
        reward_arr = np.zeros(shape=(agent.n_rollouts,1))   # Array of reward values
        theta_rl,epsilon_rl = agent.get_theta()             # Generate sample policies from distribution


        ## PRINT EPISODE DATA
        print("=============================================")
        print("STARTING Episode # %d" %k_ep)
        print("=============================================")

        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
        print(f"mu_0 = {agent.mu[0,0]:.3f}, \t sig_0 = {agent.sigma[0,0]:.3f}")
        print(f"mu_1 = {agent.mu[1,0]:.3f}, \t sig_1 = {agent.sigma[1,0]:.3f}")
        print('\n')
        

        print("theta_rl = ")
        print(theta_rl[0,:], "--> RREV")
        print(theta_rl[1,:], "--> G1")
        




        # ============================
        ##          Run 
        # ============================
        for env.k_run in range(0,env.n_rollouts):

            ## UPDATE RUN NUMBER
            k_run = env.k_run # Local variables are faster to access then class variables

            try: # Use try block to catch raised exceptions and attempt rollout again
               
            
                ## RESET TO INITIAL STATE
                env.step('home') # Reset control vals and functionality to default vals
                time.sleep(0.65) # Time for CF to settle [Real-Time seconds]

                

                ## INITIALIZE POLICY PARAMETERS: 
                #  Policy implemented in controller node (controller.cpp)
                RREV_thr = theta_rl[0, k_run] # RREV threshold (FOV expansion velocity) [rad/s]
                G1 = theta_rl[1, k_run]
                G2 = 0.0

                env.policy = [RREV_thr,np.abs(G1),G2]
                env.step('policy',env.policy,ctrl_flag=1) # Arm controller policy


                ## RESET/UPDATE RUN CONDITIONS
                env.runComplete_flag = False
                env.runReset_flag = False
                repeat_run= False

                start_time_rollout = env.getTime()
                start_time_pitch = np.nan

                ## RESET LOGGING CONDITIONS 
                env.error_str = ""
                
                t_step = 0
                t_prev = 0.0


                ## RESET IMPACT CONDITIONS
                env.impact_flag = False
                env.pad_contacts = [] # Reset impact condition variables
                env.body_contact = False
                env.ceiling_ft_z = 0.0
                env.ceiling_ft_x = 0.0

                ## RESET FLIP CONDITIONS
                env.flip_flag = False
                flag = False # Ensures flip data recorded only once (Needs a better name)

                
                ## RESET REWARD CALC VALUES
                env.z_max = 0.0
                env.pitch_sum = 0.0
                env.pitch_max = 0.0



                ## PRINT RUN CONDITIONS AND POLICY
                print(f"\n!-------------------Episode # {k_ep:d} run # {k_run:d}-----------------!")
                print(f"RREV_thr: {RREV_thr:.3f} \t Gain_1: {G1:.3f}")
                print(f"Vx_d: {env.vel_d[0]:.3f} \t Vy_d: {env.vel_d[1]:.3f} \t Vz_d: {env.vel_d[2]:.3f}")
                print("\n")


                ## CONVERT STARTING RREV VALUE -> Z_POS TO START ROLLOUT FROM
                RREV_start = 0.5
                pos_z = env.h_ceiling - env.vel_d[2]/RREV_start
                pos_z = 0.4

                # ============================
                ##          Rollout 
                # ============================

                # try:
                #     rospy.wait_for_message('/ctrl_data',CtrlData,timeout=2.0) # Wait to receive ctrl pub to run before continuing
                # except ROSException:
                #     print("No ctrl message received")
                #     repeat_run = True
                    

                env.step('pos',ctrl_flag=0)                     # Turn off pos control
                env.step('vel',env.vel_d,ctrl_flag=1)           # Set desired vel
                env.launch_IC(pos_z,env.vel_d[0]+0.03,env.vel_d[2])   # Use Gazebo to impart desired vel with extra vx to ensure -OF_y when around zero
                env.step('sticky',ctrl_flag=1)                  # Enable sticky pads
                
                while 1: # NOTE: [while 1:] is faster than [while True:]
                    
                    
                    
                    ## DEFINE CURRENT STATE
                    state = env.state_current   # Collect state values here so they are thread-safe
                    FM = np.array(env.FM)       # Motor thrust and Moments
                    
                    vx,vy,vz = state[7:10] # [vx,vy,vz]
    

                    # ============================
                    ##      Pitch Recording 
                    # ============================
                    if (env.flip_flag == True and flag == False):
                        start_time_pitch = env.getTime() # Starts countdown for when to reset run

                        # Recieve flip moments from controller and then update class var to be sent out of /rl_data topic
                        Mx_d = env.FM_flip[1] 
                        My_d = env.FM_flip[2]
                        Mz_d = env.FM_flip[3]
                        env.M_d = [Mx_d,My_d,Mz_d] # [N*mm]

                    
                        print("----- pitch starts -----")
                        print(f"vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}")
                        print(f"RREV_tr={env.RREV_tr:.3f}, OF_y_tr={env.OF_y_tr:.3f}, My_d={My_d:.3f} [N*mm]")  
                        print(f"Pitch Time: {env.state_flip[0]:.3f} [s]")
                        
                        flag = True # Turns on to make sure this only runs once per rollout

                    
                    


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

                    # IF TIME SINCE TRIGGERED PITCH EXCEEDS [1.5s]  
                    if env.flip_flag and ((env.getTime()-start_time_pitch) > (1.5)):
                        env.error_str = "Rollout Completed: Pitch Timeout"
                        print(env.error_str)

                        env.runComplete_flag = True

                    # IF POSITION FALLS BELOW FLOOR HEIGHT
                    if env.position[2] <= -8.0: # Note: there is a lag with this at high RTF
                        env.error_str = "Rollout Completed: Falling Drone"
                        print(env.error_str)

                        env.runComplete_flag = True

                    # IF TIME SINCE RUN START EXCEEDS [6.0s]
                    if (env.getTime() - start_time_rollout) > (3.0):
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
                        # print(f"z_max: {env.z_max}")
                        # print(f"pitch_sum: {env.pitch_sum}")
                        # print(f"max pitch: {env.pitch_max}")

                        reward_arr[k_run] = agent.calcReward_pureLanding(env)
                        env.reward = reward_arr[k_run,0]
                        env.reward_avg = reward_arr[np.nonzero(reward_arr)].mean()
                        env.reward_inputs = [env.z_max,env.pitch_sum,env.pitch_max]
                        
                        
                        print(f"Reward = {env.reward:.3f}")
                        # print(f"# of Leg contacts: {sum(env.pad_contacts)}")
                        print("!------------------------End Run------------------------! \n")  

                        env.RL_Publish() # Publish that rollout completed 

                        ## RUN DATA LOGGING
                        env.append_csv_blank()
                        env.append_IC()
                        env.append_flip()
                        env.append_impact()
                        env.append_csv_blank()

                        env.reset_pos()

                        env.clear_IF_Data()
                        
                    
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
                    k_run += 1 # When succesful move on to next run
                    
            except: ## IF SIM EXCEPTION RAISED THEN CONTINUE BACK TO TRY BLOCK UNTIL SUCCESSFUL COMPLETION
                pass

            
        ## =======  EPISODE COMPLETED  ======= ##
        print(f"Episode # {k_ep:d} training, average reward {env.reward_avg:.3f}")
        agent.train(theta_rl,reward_arr,epsilon_rl)

        
        
       
    ## =======  MAX TRIALS COMPLETED  ======= ##


if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    env.launch_dashboard()

    # ============================
    ##          AGENT  
    # ============================

    ## GAUSSIAN PARAMETERS
    mu = np.array([[3.0],[2.5]])                 # Initial mu starting point
    sigma = np.array([[1.5],[2.0]])       # Initial sigma starting point


    ## LEARNING AGENTS AND PARAMETERS
    env.n_rollouts = 8
    K_EP_MAX = rospy.get_param("K_EP_MAX")
    agent = rlEM_PEPGAgent(mu,sigma,n_rollouts=env.n_rollouts)

    
    # ============================
    ##     LEARNING CONDITIONS  
    # ============================

    ## CONSTANT VELOCITY LAUNCH CONDITIONS
    V_d = 2.653   # [m/s]
    phi = 90    # [deg]


    
    ## INITIALIALIZE LOGGING DATA
    trial_num = 2
    env.agent_name = agent.agent_type
    env.trial_name = f"{env.agent_name}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}--SIM"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
    env.logging_flag = True
       


    ## RUN TRIAL
    env.RL_Publish() # Publish data to rl_data topic
    time.sleep(3)

    runTraining(env,agent,V_d,phi,k_epMax=K_EP_MAX)
    print()
 
    



