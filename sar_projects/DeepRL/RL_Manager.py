## STANDARD IMPORTS
import os
from datetime import datetime,timedelta
import numpy as np
import pandas as pd
import torch as th
import yaml
import pandas as pd
import csv
import time 
import rospy
import rospkg


## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
from stable_baselines3.common import utils
from stable_baselines3.common.env_util import make_vec_env
# from stable_baselines3.common.env_checker import check_env


## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%m_%d-%H:%M")

class RL_Training_Manager():
    def __init__(self,env,log_dir,log_name,env_kwargs=None):

        self.vec_env = make_vec_env(env, env_kwargs=env_kwargs)
        self.log_dir = log_dir
        self.log_name = log_name
        self.log_subdir = os.path.join(self.log_dir, self.log_name)

        self._setup_logging()


    def _setup_logging(self):
        os.makedirs(self.log_subdir, exist_ok=True)


    def create_model(self,gamma=0.999,learning_rate=0.002,net_arch=[16,16]):

        self.model = SAC(
            "MlpPolicy",
            env=self.vec_env,
            gamma=gamma,
            learning_rate=learning_rate,
            ent_coef='auto',
            policy_kwargs=dict(activation_fn=th.nn.LeakyReLU,net_arch=dict(pi=net_arch, qf=[256,256])),
            verbose=1,
            device='cpu',
            tensorboard_log=self.log_subdir
        ) 


    def train_model(self,total_timesteps=2e6,reset_timesteps=True):

        self.model.learn(
            total_timesteps=int(total_timesteps),
            tb_log_name="TB_Log",
            reset_num_timesteps=reset_timesteps,
        )



if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
    from Envs.SAR_2D_DeepRL import SAR_2D_Env


    current_datetime = datetime.now()
    current_time = current_datetime.strftime("%H:%M:%S")
    log_name = f"DeepRL_Policy_{current_time}"
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL" 

    # ================================================================= ##

    # Define the environment parameters
    env_kwargs = {
        "Ang_Acc_range": [-100, 0],
        "V_mag_range": [2.5, 2.5],
        "V_angle_range": [60, 60],
        "Plane_Angle_range": [0, 0],
        "Render": True
    }


    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)
    RL_Manager.create_model()
    RL_Manager.train_model()
