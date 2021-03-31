#!/usr/bin/env python3


import numpy as np
import time,os



from Crazyflie_env import CrazyflieEnv
from rl_syspepg import rlsysPEPGAgent_reactive,rlsysPEPGAgent_adaptive
from rl_EM import rlEM_PEPGAgent,rlEM_AdaptiveAgent
from rl_cma import CMA_basic,CMA,CMA_sym

os.system("clear")
np.set_printoptions(precision=2, suppress=True)


def runTraining(env,agent,V_d,phi,k_epMax=500):
    env.create_csv(env.filepath)
    phi_rad = phi*np.pi/180
    
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
        reward = np.zeros(shape=(agent.n_rollouts,1))
        theta_rl,epsilon_rl = agent.get_theta()


        ## PRINT EPISODE DATA
        print("=============================================")
        print("STARTING Episode # %d" %k_ep)
        print("=============================================")

        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )

        print("RREV=%.3f, \t theta1=%.3f" %(agent.mu[0], agent.mu[1]))
        print("sig1=%.3f, \t sig2=%.3f" %(agent.sigma[0], agent.sigma[1]))
        print('\n')

        print("theta_rl = ")
        print(theta_rl[0,:], "--> RREV")
        print(theta_rl[1,:], "--> G1")
        




        # ============================
        ##          Run 
        # ============================
        k_run = 0 # Reset run counter each episode
        while k_run < env.n_rollouts:

            try:
            
                ## RESET TO INITIAL STATE
                env.step('home') # Reset control vals and functionality to default vals
                time.sleep(0.65) # Time for CF to settle
                

                ## INITIALIZE POLICY PARAMETERS: 
                #  POLICY IMPLEMENTED IN CONTROLLER NODE (CONTROLLER.CPP)
                RREV_threshold = theta_rl[0, k_run] # FoV expansion velocity [rad/s]
                G1 = theta_rl[1, k_run]
                env.policy = [RREV_threshold,np.abs(G1),0]
                env.step('policy',env.policy,ctrl_flag=1) # Arm controller policy
            

                
                ## RESET/UPDATE RUN CONDITIONS
                env.k_run = k_run   # Update class k_run variable
                env.pad_contacts = [False,False,False,False] # Reset pad contacts
                env.body_contact = False
                env.ceiling_ft_z = 0.0
                env.ceiling_ft_x = 0.0




                vy_d = 0    # [m/s]
                env.vel_d = [V_d*np.cos(phi_rad), vy_d, V_d*np.sin(phi_rad)] # [m/s]
                


                ## INIT RUN FLAGS
                env.runComplete_flag = False
                repeat_run= False

                env.impact_flag = False
                env.flip_flag = False
                flag = False # Ensures flip data printed only once (Needs a better name)
                

                start_time_rollout = env.getTime()
                start_time_pitch = np.nan
                start_time_height = env.getTime()

                state_history = None
                FM_history = None
                env.error_str = ""

                log_sample_prev = np.zeros(10)
                t_step = 0

                ## INIT REWARD CALC VALUES
                env.z_max = 0.0
                env.pitch_sum = 0.0
                env.pitch_max = 0.0



                ## PRINT RUN CONDITIONS AND POLICY
                print("\n!-------------------Episode # %d run # %d-----------------!" %(k_ep,k_run))
                print("RREV_thr: %.3f \t Gain_1: %.3f" %(RREV_threshold, G1))
                # print("Vx_d: %.3f \t Vy_d: %.3f \t Vz_d: %.3f" %(vx_d, vy_d, vz_d))

                
                # ============================
                ##          Rollout 
                # ============================
                env.step('pos',ctrl_flag=0)                     # Turn off pos control
                env.step('vel',env.vel_d,ctrl_flag=1)           # Set desired vel
                env.launch_IC(env.vel_d[0]+0.03,env.vel_d[2])   # Use Gazebo to impart desired vel with extra vx to ensure -OF_y when around zero
                env.step('sticky',ctrl_flag=1)                  # Enable sticky pads
                
                while 1: # NOTE: [while 1:] is faster than [while True:]
                    
                    
                    
                    ## DEFINE CURRENT STATE
                    state = env.state_current   # Collect state values here so they are thread-safe
                    FM = np.array(env.FM)       # Motor thrust and Moments
                    
                    vel = state[8:11] # [vx,vy,vz]
                    vx,vy,vz = vel
       
                    


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

                    
                        print('----- pitch starts -----')
                        print('vx=%.3f, vy=%.3f, vz=%.3f' %(vx,vy,vz))
                        print('RREV_tr=%.3f, OF_y_tr=%.3f, OF_x=%.3f, My_d=%.3f [N*mm]' %(env.RREV, env.OF_y, env.OF_x, My_d) )   
                        print("Pitch Time: %.3f" %start_time_pitch)

                        flag = True # Turns on to make sure this only runs once per rollout

                    
                    


                    # ============================
                    ##      Record Keeping  
                    # ============================
                    ## Keep record of state vector for reward calc
                    state = state[:, np.newaxis] # Convert [14,] array to [14,1] array
                    FM = FM[:,np.newaxis]
                    
                    
                    if state_history is None:
                        state_history = state 
                        FM_history = FM
                    else: # Append state_history columns with current state vector

                        # Check if sample of recorded data changed, if so then append csv (Reduces repeated data rows)
                        # if not np.array_equal(log_sample,log_sample_prev): 
                        if t_step%50==0: 
                            state_history = np.append(state_history, state, axis=1)
                            FM_history = np.append(FM_history,FM,axis=1)
                            env.RL_Publish()
                            env.append_csv()

                        


                    # ============================
                    ##    Termination Criteria 
                    # ============================

                    # IF TIME SINCE TRIGGERED PITCH EXCEEDS [1.0s]  
                    if env.flip_flag and ((env.getTime()-start_time_pitch) > (1.0)):
                        # I don't like this error formatting, feel free to improve on
                        env.error_str = "Rollout Completed: Pitch Timeout"
                        print(env.error_str)

                        env.runComplete_flag = True

                    # IF POSITION FALLS BELOW ACHIEVED MAX HEIGHT

                    if env.position[2] <= 0.0: # Note: there is a lag with this
                        env.error_str = "Rollout Completed: Falling Drone"
                        print(env.error_str)

                        env.runComplete_flag = True


                    if (env.getTime() - start_time_height) > (5.0):
                        env.error_str = "Rollout Completed: Pos Time Exceeded"
                        print(env.error_str)
                        env.runComplete_flag = True

                    # IF TIME SINCE RUN START EXCEEDS [2.5s]
                    if (env.getTime() - start_time_rollout) > (6.0):
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

                        print(f"z_max: {env.z_max}")
                        print(f"pitch_sum: {env.pitch_sum}")
                        print(f"max pitch: {env.pitch_max}")

                        reward[k_run] = agent.calcReward_pureLanding(env)
                        env.reward = reward[k_run,0]
                        
                        
                        print(f"Reward = {env.reward:.3f}")
                        print(f"# of Leg contacts: {sum(env.pad_contacts)}")
                        print("!------------------------End Run------------------------! \n")   

                        env.append_csv_blank()
                        env.append_IC()
                        env.append_flip()
                        env.append_impact()
                        env.append_csv_blank()

                        env.reset_pos()
                    
                        break
                    
                    # log_sample_prev = log_sample          
                    t_step += 1      
                
                ## =======  RUN COMPLETED  ======= ##
                if repeat_run == True:
                    env.relaunch_sim()
                
                else:
                    ## PUBLISH UPDATED REWARD VARIABLES
                    env.reward = reward[k_run,0]
                    env.reward_avg = reward[np.nonzero(reward)].mean()
                    env.RL_Publish()
                    k_run += 1 # Move on to next run
                    
            except FileNotFoundError: ## IF SIM EXCEPTION RAISED THEN CONTINUE BACK TO TRY BLOCK UNTIL SUCCESSFUL COMPLETION
                continue

            
        ## =======  EPISODE COMPLETED  ======= ##
        print("Episode # %d training, average reward %.3f" %(k_ep, env.reward_avg))
        agent.train(theta_rl,reward,epsilon_rl)

        
        
       
    ## =======  MAX TRIALS COMPLETED  ======= ##


if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)
    # env.launch_dashboard()

    print("Environment done")

    # ============================
    ##          AGENT  
    # ============================

    ## LEARNING RATES
    alpha_mu = np.array([[0.1]])
    alpha_sigma = np.array([[0.05]])

    ## GAUSSIAN PARAMETERS
    mu = np.array([[4.80],[5.536]])          # Random initial mu
    sigma = np.array([[0.000001],[0.000001]])       # Initial estimates of sigma:


    ## SIM PARAMETERS
    env.n_rollouts = 8
    env.gamma = 0.95
    env.h_ceiling = 3.0 # [m]


    ## LEARNING AGENTS
    agent = rlEM_PEPGAgent(mu,sigma,n_rollouts=env.n_rollouts)

    
    # ============================
    ##     LEARNING CONDITIONS  
    # ============================

    ## INITIAL CONDITIONS
    V_d = 4.0     # [m/s]
    phi = 70    # [deg]


    
    ## INITIAL LOGGING DATA
    trial_num = 3
    env.agent_name = agent.agent_type
    env.trial_name = f"{env.agent_name}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}"
    env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
    env.logging_flag = True
       


    ## RUN TRIAL
    env.RL_Publish() # Publish data to rl_data topic
    time.sleep(3)
    runTraining(env,agent,V_d,phi)
 
    



