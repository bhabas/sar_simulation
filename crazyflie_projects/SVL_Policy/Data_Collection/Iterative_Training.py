import numpy as np
import pandas as pd
import os


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)

from crazyflie_projects.SVL_Policy.Envs.CrazyflieEnv_ParamOpt import CrazyflieEnv_ParamOpt
from crazyflie_projects.SVL_Policy.RL_Training_2_term_Policy import runTraining
from crazyflie_projects.SVL_Policy.Agents.EPHE_Agent import EPHE_Agent


if __name__ == '__main__':

    ## Home Test List
    df = pd.read_csv(f"{BASE_PATH}/crazyflie_projects/SVL_Policy/Data_Collection/MasterTestList.csv")
    arr = df.to_numpy()
    # arr = np.flip(arr,axis=0)
    
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv_ParamOpt(GZ_Timeout=True)

    for V_d,phi,tau_0,trial_num in arr:
        
        
        ## INIT LEARNING AGENT
        # Mu_Tau value is multiplied by 10 so complete policy is more normalized
        mu_0 = [tau_0*10,np.random.uniform(3.0,8.0)]       # Initial mu starting point
        sig_0 = [0.5,1.5]   # Initial sigma starting point
        agent = EPHE_Agent(mu_0,sig_0,n_rollouts=6)


        ## INITIALIALIZE LOGGING DATA
        trial_name = f"{agent.agent_type}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}--{env.modelInitials()}--DR.csv"
        
        ## PARAMTER OPTIMIZATION
        runTraining(env,agent,V_d,phi,trial_name,K_ep_max=15)

        