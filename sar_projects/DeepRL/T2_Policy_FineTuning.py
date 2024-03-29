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


## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))




if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
    from Envs.SAR_2D_DeepRL import SAR_2D_Env

    # ================================================================= ##

    # Define the environment parameters
    env_kwargs = {
        "Ang_Acc_range": [-100, 0],
        "V_mag_range": [1.5,4.5],
        "V_angle_range": [10,170],
        "Plane_Angle_range": [0,135],
        "Render": False,
        "Fine_Tune": False,
    }
    

    current_datetime = datetime.now()
    current_time = current_datetime.strftime("%m-%d--%H:%M:%S")
    Log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    Log_name = f"DeepRL_Policy_{current_time}"

    RL_Manager = RL_Training_Manager(SAR_2D_Env,Log_dir,Log_name,env_kwargs=env_kwargs)


    model_kwargs = {
        "gamma": 0.999,
        "learning_rate": 0.002,
        "net_arch": dict(pi=[10,10,10], qf=[64,64,64]),
        "ent_coef": "auto",
        "target_entropy": "auto",
        "batch_size": 256,
        "buffer_size": int(20e3),
    }

    RL_Manager.create_model(model_kwargs)
    
    Model_to_Load = "DeepRL_Policy_ent_0.02_fixed_03-29--09:29:02"
    RL_Manager.load_model(t_step=255500,Log_name=Model_to_Load,Params_only=True,load_replay_buffer=False)
    ## CHANGE ENTROPY COEFFICIENT ON LOADING
    RL_Manager.train_model(reset_timesteps=True)
    # RL_Manager.sweep_policy(Plane_Angle_Step=45,V_mag_Step=1.0,V_angle_Step=10,n=2)