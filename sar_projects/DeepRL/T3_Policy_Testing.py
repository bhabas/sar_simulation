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
        "V_angle_range": [15,165],
        "Plane_Angle_range": [0,0],
        "Render": True,
    }


    
    log_name = "A30_L200_0deg_BaseTest_03-30--15:04:50"
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)

    RL_Manager.load_model(t_step=113500,Log_name=log_name,Params_only=False)
    RL_Manager.sweep_policy(Plane_Angle_Step=45,V_mag_Step=1.0,V_angle_Step=15,n=2)
