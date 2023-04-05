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
    env = CrazyflieEnv_DeepRL_LDA(GZ_Timeout=True,Vel_range=[0.5,3.5],Phi_range=[30,90])
    
    log_dir = f"{BASE_PATH}/crazyflie_projects/Leg_Design_Analysis/TB_Logs/{env.env_name}"

    ## CREATE NEW DEEP RL MODEL 
    # log_name = f"SAC--{current_time}--{env.modelInitials}"    
    # PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    # PolicyTrainer.create_model()
    # PolicyTrainer.train_model()
    
    # LOAD DEEP RL MODEL
    log_name = "SAC--01_25-20:21--LDA_A60_L100_K64_0"
    t_step_load = 91000

    PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    PolicyTrainer.load_model(t_step_load)
    PolicyTrainer.train_model()

    # ================================================================= ##

    # COLLECT LANDING PERFORMANCE DATA
    # log_dir = f"{BASE_PATH}/crazyflie_projects/Leg_Design_Analysis/TB_Logs/{env.env_name}"
    # log_name = "SAC--02_25-22:03--LDA_A30_L100_K08_0" #### CHANGE MODEL IN SIM SETTINGS TO MATCH
    # ############ DID YOU FORGET TO CHANGE MODEL IN SIM SETTINGS
    # #########
    # ############ I HOPE YOU DIDN'T
    # t_step_load = 110000

    # PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    # PolicyTrainer.load_model(t_step_load)
    # PolicyTrainer.test_landing_performance()

    # ================================================================= ##

    # # PLOT LANDING PERFORMANCE
    # env = None
    # log_dir = f"{BASE_PATH}/crazyflie_projects/Leg_Design_Analysis/TB_Logs/CF_Gazebo"
    # log_name = "SAC--02_25-22:03--LDA_A30_L100_K08_0"
    # PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    # PolicyTrainer.Plot_Landing_Performance(saveFig=True)