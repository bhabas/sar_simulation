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
        "Plane_Angle_range": [0,180],
        "Render": False,
    }


    
    log_name = "DeepRL_Policy_10x3_Ceiling_03-27--22:19:29"
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)
    # RL_Manager.load_model(t_step=88000,Log_name=log_name,Params_only=False)

    # RL_Manager.collect_landing_performance(
    #     fileName="PolicyPerformance_Data_473000.csv",
    #     Plane_Angle_range=[0,180,45],
    #     V_mag_range=[1.5,4.5,0.5],
    #     V_angle_range=[15,165,5],
    #     n_trials=5
    #     )
    RL_Manager.plot_landing_performance(fileName="PolicyPerformance_Data_473000.csv",PlaneAngle=180,saveFig=True)