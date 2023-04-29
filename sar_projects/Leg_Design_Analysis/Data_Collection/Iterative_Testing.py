import numpy as np
import pandas as pd
import os


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,BASE_PATH)

from crazyflie_env.CrazyflieEnv_ParamOpt import CrazyflieEnv_ParamOpt
from crazyflie_env.RL_Training_2_term_Policy import runTraining
from crazyflie_env.RL_agents.EPHE_Agent import EPHE_Agent


if __name__ == '__main__':

    ## Home Test List
    df = pd.read_csv(f"{BASE_PATH}/crazyflie_projects/DeepRL/Data_Collection/MasterTestList.csv")
    arr = df.to_numpy()
    # arr = np.flip(arr,axis=0)
    
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv_ParamOpt(GZ_Timeout=True)

    for V_d,phi in arr:
        
        
        ## INIT LEARNING AGENT
        mu_0 = [5,5]       # Initial mu starting point
        sig_0 = [1.5,1.5]   # Initial sigma starting point
        agent = EPHE_Agent(mu_0,sig_0,n_rollouts=5)


        ## INITIALIALIZE LOGGING DATA
        trial_name = f"DeepRL--Vd_{V_d:.2f}--phi_{phi:.2f}--{env.modelInitials}.csv"

        
        ## PARAMTER OPTIMIZATION
        runTraining(env,agent,V_d,phi,trial_name,K_ep_max=1)

        
