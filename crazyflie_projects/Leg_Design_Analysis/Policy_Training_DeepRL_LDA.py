## STANDARD IMPORTS
import os
from datetime import datetime
import numpy as np
import pandas as pd
import torch as th

## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,BASE_PATH)


## IMPORT ENVIRONMENTS
from crazyflie_projects.DeepRL.Policy_Training_DeepRL import Policy_Trainer_DeepRL
from crazyflie_projects.Leg_Design_Analysis.Envs.CrazyflieEnv_DeepRL_LDA import CrazyflieEnv_DeepRL_LDA


## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%m_%d-%H:%M")



if __name__ == '__main__':

    ## INITIATE ENVIRONMENT
    env = CrazyflieEnv_DeepRL_LDA(GZ_Timeout=True)


    # ## CREATE NEW DEEP RL MODEL 
    log_dir = f"{BASE_PATH}/crazyflie_projects/Leg_Design_Analysis/TB_Logs/"
    log_name = f"SAC--{current_time}--{env.modelInitials}"
    model = SAC(
        "MlpPolicy",
        env=env,
        gamma=0.999,
        learning_rate=0.002,
        policy_kwargs=dict(activation_fn=th.nn.ReLU,net_arch=[12,12]),
        verbose=1,
        device='cpu',
        tensorboard_log=log_dir
    ) 

    
    PolicyTrainer = Policy_Trainer_DeepRL(env,model,log_dir,log_name)
    PolicyTrainer.train_model()
