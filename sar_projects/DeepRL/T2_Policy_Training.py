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
        "V_mag_range": [1.5,4.0],
        "V_angle_range": [10,170],
        "Plane_Angle_range": [45,45],
        "Render": False,
    }
    

    current_datetime = datetime.now()
    current_time = current_datetime.strftime("%m-%d--%H:%M:%S")
    Log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    Log_name = f"DeepRL_Policy_{current_time}"

    RL_Manager = RL_Training_Manager(SAR_2D_Env,Log_dir,Log_name,env_kwargs=env_kwargs)
    RL_Manager.create_model(net_arch=[64,64,64])

    
    Model_to_Load = "DeepRL_Policy_03-27--07:57:57"
    RL_Manager.load_model(t_step=63500,Log_name=Model_to_Load,Params_only=True)
    RL_Manager.train_model(reset_timesteps=True)
    # RL_Manager.sweep_policy(Plane_Angle_Step=45,V_mag_Step=1.0,V_angle_Step=10,n=2)