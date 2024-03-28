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


    current_datetime = datetime.now()
    current_time = current_datetime.strftime("%m-%d--%H:%M:%S")
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    log_name = f"DeepRL_Policy_10x3_Ceiling_{current_time}"

    # ================================================================= ##

    # Define the environment parameters
    env_kwargs = {
        "Ang_Acc_range": [-100, 0],
        "V_mag_range": [1.5,4.5],
        "V_angle_range": [10,170],
        "Plane_Angle_range": [0,180],
        "Render": False,
    }
    
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)
    RL_Manager.create_model(net_arch=[10,10,10])
    RL_Manager.train_model(reset_timesteps=False)