from RL_Manager import RL_Training_Manager

## STANDARD IMPORTS
import os
from datetime import datetime,timedelta
import numpy as np
import pandas as pd
import yaml
import pandas as pd
import csv
import time 
import rospy
import rospkg
import glob
from typing import Callable


## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))



if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
    from Envs.SAR_2D_DeepRL import SAR_2D_Env


    current_datetime = datetime.now()
    current_time = current_datetime.strftime("%m-%d--%H:%M:%S")
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    log_name = input("Enter the name of the log file: ")
    log_name = f"{log_name}_{current_time}"
    # log_name = "SOV5_A45_L150_0deg_S2D_PreTraining_Agent_04-03--08:56:07"

    # ================================================================= ##

    # Define the environment parameters
    env_kwargs = {
        "Ang_Acc_range": [-90, 0],
        "V_mag_range": [0.4,5.1],
        "V_angle_range": [5,135],
        "Plane_Angle_range": [45,45],
        "Render": False,
    }
    
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs,S3_Upload=False)

    model_kwargs = {
        "gamma": 0.999,
        "learning_rate": 2.0e-3,
        "net_arch": dict(pi=[10,10,10], qf=[64,64,64]),
        "ent_coef": "auto_0.005",
        "target_entropy": -2,
        "batch_size": 256,
        "buffer_size": int(200e3),
    }

    RL_Manager.create_model(model_kwargs)

    # Model_to_Load = "SOV5_A45_L150_0deg_S2D_PreTraining_Agent"
    # RL_Manager.load_model(t_step=125e3,Log_name=Model_to_Load,Params_only=True,load_replay_buffer=False)
    RL_Manager.train_model(reset_timesteps=False,total_timesteps=int(126e3))

