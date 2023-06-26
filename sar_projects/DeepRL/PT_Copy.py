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

## PLOTTING IMPORTS
import matplotlib.pyplot as plt
import matplotlib as mpl
import plotly.graph_objects as go
from scipy.interpolate import griddata


## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
from stable_baselines3.common import utils

## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))



class Policy_Trainer_DeepRL():
    def __init__(self,env,log_dir,log_name):
        self.env = env
        self.log_dir = log_dir

        ## LOADED MODEL
        if log_name[-2] == "_": 
            self.log_name = log_name[:-2]
            latest_run_id = utils.get_latest_run_id(log_dir,log_name)
            self.TB_log_path = os.path.join(log_dir,f"{self.log_name}_{latest_run_id}")
            self.model_dir = os.path.join(self.TB_log_path,"models")

        ## NEW MODEL
        else: 
            self.log_name = log_name
            latest_run_id = utils.get_latest_run_id(log_dir,log_name)
            self.TB_log_path = os.path.join(log_dir,f"{self.log_name}_{latest_run_id}")
            self.model_dir = os.path.join(self.TB_log_path,"models")

        ## GENERATE LOG/MODEL DIRECTORY
        if not os.path.exists(self.TB_log_path):
            os.makedirs(self.TB_log_path,exist_ok=True)

        if not os.path.exists(self.model_dir):
            os.makedirs(self.model_dir,exist_ok=True)

    def create_model(self,gamma=0.999,learning_rate=0.002,net_arch=[8,8]):
        """Create Soft Actor-Critic agent used in training

        Args:
            gamma (float, optional): Discount factor. Defaults to 0.999.
            learning_rate (float, optional): Learning Rate. Defaults to 0.002.
            net_arch (list, optional): Network layer sizes and architechure. Defaults to [12,12].
        """        

        self.model = SAC(
            "MlpPolicy",
            env=self.env,
            gamma=gamma,
            learning_rate=learning_rate,
            policy_kwargs=dict(activation_fn=th.nn.ReLU,net_arch=dict(pi=net_arch, qf=[256,256])),
            verbose=1,
            device='cpu',
            tensorboard_log=self.log_dir
        ) 

        self.save_config_file()

        return self.model
    
    def train_model(self,total_timesteps=2e6,save_freq=200,reset_timesteps=False):
        """Script to train model via Deep RL method

        Args:
            log_name (str): _description_
            reset_timesteps (bool, optional): Reset starting timestep to zero. Defaults to True. 
            Set to False to resume training from previous model.
        """       

        # checkpoint_callback = CheckpointSaveCallback(save_freq=save_freq,model_dir=self.model_dir)
        self.model.predict(np.array([0.25,0,0.5],dtype=np.float32))
        # self.model.learn(
        #     total_timesteps=int(total_timesteps),
        #     tb_log_name=self.log_name,
        #     # callback=checkpoint_callback,
        #     reset_num_timesteps=reset_timesteps,
        #     progress_bar=True
        # )

    def save_config_file(self):

        config_path = os.path.join(self.TB_log_path,"Config.yaml")

        data = dict(
            PLANE_SETTINGS = dict(
                Plane_Model = self.env.Plane_Model,
                Plane_Angle = self.env.Plane_Angle,
                Plane_Pos = dict(
                    X = self.env.Plane_Pos[0],
                    Y = self.env.Plane_Pos[1],
                    Z = self.env.Plane_Pos[2]
                ),
            ),

            SAR_SETTINGS = dict(
                SAR_Type = self.env.SAR_Type,
                SAR_Config = self.env.SAR_Config,
            ),

            ENV_SETTINGS = dict(
                Environment = self.env.Env_Name,
                Vel_Limts = self.env.Vel_range,
                Phi_Limits = self.env.Phi_range,
            ),

            LEARNING_MODEL = dict(
                Policy = self.model.policy.__class__.__name__,
                Observation_Layer = self.model.policy.observation_space.shape[0],
                Network_Layers = self.model.policy.net_arch,
                Action_Layer = self.model.policy.action_space.shape[0]*2,
                Action_Space_High = self.model.policy.action_space.high.tolist(),
                Action_Space_Low = self.model.policy.action_space.low.tolist(),
                Gamma = self.model.gamma,
                Learning_Rate = self.model.learning_rate,
                Activation_Function = "",
            )


        )

        with open(config_path, 'w') as outfile:
            yaml.dump(data,outfile,default_flow_style=False,sort_keys=False)




if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL2 import SAR_Sim_DeepRL


    # # START TRAINING NEW DEEP RL MODEL 
    env = SAR_Sim_DeepRL(GZ_Timeout=False,Vel_range=[1.0,3.0],Phi_range=[0,90])
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/{env.Env_Name}"
    log_name = f"Test_Log1"    

    PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    PolicyTrainer.create_model()
    PolicyTrainer.train_model()