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
        "Ang_Acc_range": [-90, 0],
        "Render": False,
    }

    Model_to_Load = "SOV5_A30_L200_90deg_S2D_PreTraining_Agent"
    t_step = 173500
    PlaneAngle = 90

    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,Model_to_Load,env_kwargs=env_kwargs)
    RL_Manager.create_model(write_config=False)
    RL_Manager.load_model(t_step=t_step,Log_name=Model_to_Load,Params_only=True,load_replay_buffer=False)

    RL_Manager.collect_landing_performance(
        fileName="PolicyPerformance_Data.csv",
        Plane_Angle_range=[PlaneAngle,PlaneAngle,180],
        V_mag_range=[0.5,5.0,0.5],
        V_angle_range=[10,170,5],
        n_trials=5
        )
    

    RL_Manager.plot_landing_performance(fileName="PolicyPerformance_Data.csv",PlaneAngle=PlaneAngle,saveFig=True,showFig=False)
