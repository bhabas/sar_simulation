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

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%m_%d-%H:%M")

class CheckpointSaveCallback(BaseCallback):

    def __init__(self, save_freq: int, model_dir, verbose: int = 0):
        super(CheckpointSaveCallback, self).__init__(verbose)
        self.save_freq = save_freq
        self.model_dir = model_dir

        self.t_step_CircBuff = [0,0,0,0,0] # Circular buffer of recent timesteps

        
    def _on_step(self) -> bool:

        ## SAVE MODEL AND REPLAY BUFFER ON SAVE_FREQ
        if self.num_timesteps % self.save_freq == 0:

            ## APPEND NEWEST TIMESTEP
            self.t_step_CircBuff.append(self.num_timesteps)

            ## SAVE NEWEST MODEL AND REPLAY BUFFER
            newest_model = os.path.join(self.model_dir, f"{self.num_timesteps}_step_model")
            newest_replay_buff = os.path.join(self.model_dir, f"{self.num_timesteps}_step_replay_buff")

            self.model.save(newest_model)
            self.model.save_replay_buffer(newest_replay_buff)


            ## DELETE OLDEST MODEL AND REPLAY BUFFER
            oldest_model = os.path.join(self.model_dir,f"{self.t_step_CircBuff[0]}_step_model.zip")
            oldest_replay_buff = os.path.join(self.model_dir,f"{self.t_step_CircBuff[0]}_step_replay_buff.pkl")

            if os.path.exists(oldest_model):
                os.remove(oldest_model)

            if os.path.exists(oldest_replay_buff):
                os.remove(oldest_replay_buff)

            ## REMOVE OLDEST TIMESTEP
            self.t_step_CircBuff.pop(0)


            if self.verbose > 1:
                print(f"Saving model checkpoint to {newest_model}")
        return True

    def _on_rollout_end(self) -> None:
        self.logger.record('time/K_ep',self.training_env.envs[0].env.K_ep)
        return True
    
class RewardCallback(BaseCallback):
    def __init__(self, check_freq: int, verbose=1):
        super(RewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.rewards = []
        self.episode_rewards = []

    def _on_step(self) -> bool:
        # This method will be called by the model after each call to env.step().

        # We assume the environments are VecEnv, so they are arrays
        if self.locals["dones"].any():
            # Get the episodic reward for each environment
            episode_rewards = self.locals["rewards"]
            self.episode_rewards.append(episode_rewards)

            if len(self.episode_rewards) > self.check_freq:
                # Compute and store the mean episodic reward
                mean_reward = sum(self.episode_rewards) / len(self.episode_rewards)
                self.rewards.append(mean_reward)

                print(f"Reward Avg: {mean_reward[0]:.3f}")

                # Reset the episodic rewards
                self.episode_rewards = []

        return True

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

        # self.save_config_file()

        return self.model
    
    def load_model(self,t_step):
        """Loads current model and replay buffer from the selected time step

        Args:
            t_step (int): Timestep of the selected model
        """        

        ## MODEL PATHS
        model_path = os.path.join(self.model_dir,f"{t_step}_step_model")
        replay_buff_path = os.path.join(self.model_dir,f"{t_step}_step_replay_buff")

        ## LOAD MODEL AND REPLAY BUFFER
        self.model = SAC.load(
            model_path,
            env=self.env,
            device='cpu',
            tensorboard_log=self.log_dir
        )
        self.model.load_replay_buffer(replay_buff_path)

    def load_prev_model_params(self,prev_model_dir,prev_model_name,t_step):

        ## MODEL PATHS
        prev_model_path = os.path.join(prev_model_dir,prev_model_name,"models",f"{t_step}_step_model")
        prev_replay_buff_path = os.path.join(prev_model_dir,prev_model_name,"models",f"{t_step}_step_replay_buff")

        ## LOAD PREV MODEL AND REPLAY BUFFER
        prev_model = SAC.load(
            prev_model_path,
            env=None,
            device='cpu',
        )
        self.model.load_replay_buffer(prev_replay_buff_path)

        ## TRANSFER PARAMETERS TO CURRENT MODEL
        self.model.policy.actor.parameters = prev_model.actor.parameters
        self.model.policy.critic.parameters = prev_model.critic.parameters

    
    def train_model(self,total_timesteps=2e6,save_freq=200,reset_timesteps=False):
        """Script to train model via Deep RL method

        Args:
            log_name (str): _description_
            reset_timesteps (bool, optional): Reset starting timestep to zero. Defaults to True. 
            Set to False to resume training from previous model.
        """       

        checkpoint_callback = CheckpointSaveCallback(save_freq=save_freq,model_dir=self.model_dir)
        reward_callback = RewardCallback(check_freq=20)
        callback = CallbackList([checkpoint_callback, reward_callback])

        self.model.learn(
            total_timesteps=int(total_timesteps),
            tb_log_name=self.log_name,
            callback=callback,
            reset_num_timesteps=reset_timesteps,
        )

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
    # from Envs.SAR_Sim_DeepRL2 import SAR_Sim_DeepRL
    from Envs.CF_Env_2D_2 import CF_Env_2D



    # # START TRAINING NEW DEEP RL MODEL 
    # env = SAR_Sim_DeepRL(GZ_Timeout=False,Vel_range=[1.0,3.0],Phi_range=[0,90])
    env = CF_Env_2D(Vel_range=[1.0,3.0],Phi_range=[0,90])
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/{env.Env_Name}"
    log_name = f"Test_Log_Cur2"    

    PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    PolicyTrainer.create_model()
    PolicyTrainer.load_prev_model_params(log_dir,"Test_Log_Prev_0",t_step=98200)
    PolicyTrainer.train_model()

