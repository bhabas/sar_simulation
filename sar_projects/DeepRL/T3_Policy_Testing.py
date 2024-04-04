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

    env_kwargs = {
        "Ang_Acc_range": [-90, 0],
        "V_mag_range": [0.5,5.0],
        "V_angle_range": [10,170],
        "Plane_Angle_range": [90,90],
        "Render": True,
    }

    Model_to_Load = "SOV5_A45_L150_90deg_S2D_PreTraining_Agent_04-03--18:11:41"
    t_step = 5500

    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,Model_to_Load,env_kwargs=env_kwargs)
    RL_Manager.create_model(write_config=False)
    RL_Manager.load_model(t_step=t_step,Log_name=Model_to_Load,Params_only=True,load_replay_buffer=False)
    RL_Manager.sweep_policy(V_mag_Step=1.0,V_angle_Step=10,n=1)
