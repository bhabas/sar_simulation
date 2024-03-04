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


## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))

## COLLECT CURRENT TIME
current_datetime = datetime.now()
current_time = current_datetime.strftime("%m_%d-%H:%M")

class RL_Training_Manager():
    def __init__(self,env,log_dir,log_name,env_kwargs=None):

        self.vec_env = make_vec_env(env, env_kwargs=env_kwargs)

        self.log_name = log_name
        self.log_dir = os.path.join(log_dir, log_name)
        self.model_dir = os.path.join(log_dir, log_name, "Models")

        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.model_dir, exist_ok=True)




    def create_model(self,gamma=0.999,learning_rate=0.002,net_arch=[12,12,12]):

        self.model = SAC(
            "MlpPolicy",
            env=self.vec_env,
            gamma=gamma,
            learning_rate=learning_rate,
            ent_coef='auto',
            policy_kwargs=dict(activation_fn=th.nn.LeakyReLU,net_arch=dict(pi=net_arch, qf=[256,256,256])),
            verbose=1,
            device='cpu',
            tensorboard_log=self.log_dir
        )
        
    def load_model(self,model_dir,t_step: int):

        model_path = os.path.join(model_dir, f"model_{int(t_step)}_steps")
        replay_buffer_path = os.path.join(model_dir, f"replay_buffer_{int(t_step)}_steps")

        ## LOAD MODEL AND REPLAY BUFFER
        self.model = SAC.load(
            model_path,
            env=self.vec_env,
            device='cpu',
            tensorboard_log=self.log_dir,
        )
        self.model.load_replay_buffer(replay_buffer_path)

    def train_model(self,check_freq=10,save_freq=1e3,reset_timesteps=True,total_timesteps=2e6):

        reward_callback = RewardCallback(check_freq=check_freq,save_freq=save_freq,model_dir=self.model_dir)

        self.model.learn(
            total_timesteps=int(total_timesteps),
            callback=reward_callback,
            tb_log_name="TB_Log",
            reset_num_timesteps=reset_timesteps,
        )

    def test_policy(self,V_mag=None,V_angle=None,Plane_Angle=None,episodes=10):

        # self.vec_env.Render_Flag = True
        obs = self.vec_env.reset()
        terminated = False
 
        while not terminated:
            action,_ = self.model.predict(obs)
            obs,reward,terminated,_ = self.vec_env.step(action)

        return obs,reward

class RewardCallback(BaseCallback):
    def __init__(self, check_freq: int, save_freq: int, model_dir: str, verbose=1):
        super(RewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.save_freq = save_freq
        self.model_dir = model_dir

    def _on_training_start(self) -> None:
        """
        This method is called before the first rollout starts.
        """
        pass

    def _on_step(self) -> bool:

        ## SAVE MODEL EVERY N TIMESTEPS
        if self.n_calls % self.save_freq == 0:

            ## SAVE NEWEST MODEL AND REPLAY BUFFER
            model_path = os.path.join(self.model_dir, f"model_{self.num_timesteps}_steps")
            replay_buffer_path = os.path.join(self.model_dir, f"replay_buffer_{self.num_timesteps}_steps")

            self.model.save(model_path)
            self.model.save_replay_buffer(replay_buffer_path)
        
        return True
    
    def _on_rollout_start(self) -> None:
        """
        A rollout is the collection of environment interaction
        using the current policy.
        This event is triggered before collecting new samples.
        """
        pass
    
    def _on_rollout_end(self) -> None:
        """
        This event is triggered before updating the policy.
        """
        pass



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


    log_name = "DeepRL_Policy_09:02:31"
    model_dir = "/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL/DeepRL_Policy_09:02:31/Models"
    
    RL_Manager = RL_Training_Manager(SAR_Sim_DeepRL,log_dir,log_name,env_kwargs=env_kwargs)
    # RL_Manager.create_model()
    RL_Manager.load_model(model_dir,t_step=20e3)
    RL_Manager.test_policy()
    # RL_Manager.train_model()
