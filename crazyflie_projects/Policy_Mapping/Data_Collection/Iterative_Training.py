import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import time
import rospy
import threading


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)

from crazyflie_env.src.ExecuteFlight import executeFlight
from crazyflie_env.src.Crazyflie_env import CrazyflieEnv
from crazyflie_env.src.RL_agents.rl_EM import rlEM_PEPGAgent

def runTraining(env,agent):

    # ============================
    ##          Episode         
    # ============================

    for k_ep in range(0,rospy.get_param("K_EP_MAX")):

        ## UPDATE EPISODE NUMBER
        env.k_ep = k_ep

        ## CONVERT AGENT ARRAYS TO LISTS FOR PUBLISHING
        env.mu = agent.mu.flatten().tolist()                # Mean for Gaussian distribution
        env.sigma = agent.sigma.flatten().tolist()          # Standard Deviation for Gaussian distribution

        env.mu_1_list.append(env.mu[0])
        env.mu_2_list.append(env.mu[1])

        env.sigma_1_list.append(env.sigma[0])
        env.sigma_2_list.append(env.sigma[1])

        
        ## PRE-ALLOCATE REWARD VEC AND OBTAIN THETA VALS
        training_arr = np.zeros(shape=(agent.n_rollouts,1)) # Array of reward values for training
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
        print(theta_rl[0,:], "--> Tau")
        print(theta_rl[1,:], "--> My")

        # ============================
        ##          Run 
        # ============================
        for env.k_run in range(0,env.n_rollouts):
            # input("press enter")


            ## UPDATE RUN NUMBER
            k_run = env.k_run # Local variables are faster to access then class variables


            ## INITIALIZE POLICY PARAMETERS: 
            Tau_thr = theta_rl[0, k_run]    # Tau threshold 10*[s]
            My = theta_rl[1, k_run]         # Policy Moment Action [N*mm]
            G2 = 0.0                        # Deprecated policy term
            
            env.policy = [Tau_thr/10,np.abs(My),G2]

            try: # Use try block to catch raised exceptions and attempt rollout again
                env.Iyy = rospy.get_param("Iyy") + np.random.normal(0,1.5e-6)
                env.mass = rospy.get_param("/CF_Mass") + np.random.normal(0,0.0005)
                env.updateInertia()
                executeFlight(env,agent)

                if env.repeat_run == True: # Runs when error detected
                    env.relaunch_sim()
                    continue

            except rospy.service.ServiceException:
                continue

            ## ADD VALID REWARD TO TRAINING ARRAY
            training_arr[k_run] = env.reward

            env.reward_list.append(np.round(env.reward,3))
            env.reward_avg = training_arr[np.nonzero(training_arr)].mean()

            ## PUBLISH UPDATED REWARD VARIABLES
            env.RL_Publish()

        env.reward_avg_list.append(env.reward_avg)
        env.RL_Publish()

        ## =======  EPISODE COMPLETED  ======= ##
        print(f"Episode # {k_ep:d} training, average reward {env.reward_avg:.3f}")
        agent.train(theta_rl,training_arr,epsilon_rl)

if __name__ == '__main__':
    
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)

    ## Home Test List
    df = pd.read_csv(f"{BASE_PATH}/crazyflie_projects/Policy_Mapping/Data_Collection/MasterTestList.csv")
    arr = df.to_numpy()

    for V_d,phi,tau_0,trial_num in arr:
        
        mu = [tau_0*10,np.random.uniform(3.0,8.0)]       # Initial mu starting point
        sigma = [0.5,1.5]   # Initial sigma starting point
        agent = rlEM_PEPGAgent(mu,sigma,n_rollouts=env.n_rollouts)


        ## CONSTANT VELOCITY LAUNCH CONDITIONS
        phi_rad = np.radians(phi)
        env.vel_d = [V_d*np.cos(phi_rad), 0.0, V_d*np.sin(phi_rad)] # [m/s]

        ## INITIALIALIZE LOGGING DATA
        env.agent_name = agent.agent_type
        env.trialComplete_flag = False
        env.trial_name = f"{env.agent_name}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}--{env.modelInitials()}--DR"
        env.filepath = f"{env.loggingPath}/{env.trial_name}.csv"
        env.Logging_Flag = True
        env.createCSV(env.filepath)

        runTraining(env,agent)
        env.trialComplete_flag = True
        env.RL_Publish()

        ## CLEAR OUT REWARD DATA BUFFERS
        env.mu_1_list = []
        env.mu_2_list = []
        env.sigma_1_list = []
        env.sigma_2_list = []
        env.reward_list = []
        env.reward_avg_list = []

        