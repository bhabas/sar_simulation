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
        "Render": False,
    }

    model_kwargs = {
        "gamma": 0.999,
        "learning_rate": 2e-3,
        "net_arch": dict(pi=[10,10,10], qf=[64,64,64]),
        "ent_coef": "auto_0.005",
        "target_entropy": -2,
        "batch_size": 256,
        "buffer_size": int(100e3),
    }


    
    log_name = "SOV5_A30_L200_0deg_5.0K_S3D_04-01--20:35:23"
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    RL_Manager = RL_Training_Manager(SAR_Sim_DeepRL,log_dir,log_name,env_kwargs=env_kwargs)
    RL_Manager.create_model(model_kwargs)
    RL_Manager.load_model(t_step=25000,Log_name=log_name,Params_only=True,load_replay_buffer=False)

    RL_Manager.collect_landing_performance(
        fileName="PolicyPerformance_Data.csv",
        Plane_Angle_range=[0,0,45],
        V_mag_range=[1.5,4.5,0.5],
        V_angle_range=[10,90,10],
        n_trials=3
        )
    
    RL_Manager.plot_landing_performance(fileName="PolicyPerformance_Data.csv",PlaneAngle=0,saveFig=True,showFig=False)
    # RL_Manager.plot_landing_performance(fileName="PolicyPerformance_Data.csv",PlaneAngle=45,saveFig=True,showFig=False)
    # RL_Manager.plot_landing_performance(fileName="PolicyPerformance_Data.csv",PlaneAngle=90,saveFig=True,showFig=False)
    # RL_Manager.plot_landing_performance(fileName="PolicyPerformance_Data.csv",PlaneAngle=135,saveFig=True,showFig=False)