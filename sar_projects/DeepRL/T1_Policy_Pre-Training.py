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

def const_schedule(initial_value: float) -> Callable[[float], float]:

    def func(progress_remaining: float) -> float:

        return initial_value

    return func

def step_schedule(initial_value: float, change_value: float, change_point: float) -> Callable[[float], float]:

    def func(progress_remaining: float) -> float:

        if progress_remaining > change_point:
            return initial_value
        else:
            return change_value

    return func





if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
    from Envs.SAR_2D_DeepRL import SAR_2D_Env


    current_datetime = datetime.now()
    current_time = current_datetime.strftime("%m-%d--%H:%M:%S")
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    log_name = input("Enter the name of the log file: ")
    log_name = f"{log_name}_{current_time}"

    # ================================================================= ##

    # Define the environment parameters
    env_kwargs = {
        "Ang_Acc_range": [-100, 0],
        "V_mag_range": [1.3,5.2],
        "V_angle_range": [10,90],
        "Plane_Angle_range": [0,0],
        "Render": False,
        "Fine_Tune": False,
    }
    
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)

    model_kwargs = {
        "gamma": 0.999,
        "learning_rate": 2e-3,
        "net_arch": dict(pi=[10,10,10], qf=[64,64,64]),
        "ent_coef": "auto_0.005",
        "target_entropy": -2,
        "batch_size": 256,
        "buffer_size": int(100e3),
    }

    RL_Manager.create_model(model_kwargs)

    Model_to_Load = "A30_L200_0deg_PreTraining_Agent"
    RL_Manager.load_model(t_step=55500,Log_name=Model_to_Load,Params_only=True,load_replay_buffer=False)
    RL_Manager.train_model(reset_timesteps=False,total_timesteps=int(151e3))

    RL_Manager.collect_landing_performance(
        fileName="PolicyPerformance_Data.csv",
        Plane_Angle_range=[0,0,45],
        V_mag_range=[1.6,4.4,0.4],
        V_angle_range=[15,165,10],
        n_trials=4
        )
    
    RL_Manager.plot_landing_performance(fileName="PolicyPerformance_Data.csv",PlaneAngle=0,saveFig=True,showFig=False)
    # RL_Manager.plot_landing_performance(fileName="PolicyPerformance_Data.csv",PlaneAngle=45,saveFig=True,showFig=False)
    # RL_Manager.plot_landing_performance(fileName="PolicyPerformance_Data.csv",PlaneAngle=90,saveFig=True,showFig=False)
    # RL_Manager.plot_landing_performance(fileName="PolicyPerformance_Data.csv",PlaneAngle=135,saveFig=True,showFig=False)