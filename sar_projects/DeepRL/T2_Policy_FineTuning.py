## IMPORT ENVIRONMENTS
from RL_Manager import RL_Training_Manager
from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
from Envs.SAR_2D_DeepRL import SAR_2D_Env

## STANDARD IMPORTS
import os
from datetime import datetime
import rospkg
import argparse
import json

## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))
LOG_DIR = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 



## ARGUMENT PARSER
parser = argparse.ArgumentParser(description='Policy Pre-Training Script')
parser.add_argument('--TrainConfig', help='Path to training config file', required=True)
parser.add_argument('--PreTrainConfig', help='Path to pretrained config file', required=True)
parser.add_argument('--S3_Upload', help='Upload to S3', default=False, type=bool)
args = parser.parse_args()

## LOAD TRAINING MODEL CONFIGURATION
with open(args.TrainConfig, 'r') as TrainConfig_File:
    TrainConfig = json.load(TrainConfig_File)

## LOAD PRETRAINED MODEL CONFIGURATION
with open(args.PreTrainConfig, 'r') as PreTrainConfig_File:
    PreTrainConfig = json.load(PreTrainConfig_File)

## UPDATE SAR TYPE AND SAR CONFIG IN BASE SETTINGS FILE
Base_Settings_yaml = f"{BASE_PATH}/sar_config/Base_Settings.yaml"
with open(Base_Settings_yaml, 'r') as file:
    lines = file.readlines()

for i, line in enumerate(lines):
    if line.strip().startswith('SAR_Type:'):
        lines[i] = f"  SAR_Type: '{TrainConfig['SAR_SETTINGS']['SAR_Type']}'\n"
    elif line.strip().startswith('SAR_Config:'):
        lines[i] = f"  SAR_Config: '{TrainConfig['SAR_SETTINGS']['SAR_Config']}'\n"

with open(Base_Settings_yaml, 'w') as file:
    file.writelines(lines)


if __name__ == '__main__':

    ## LOGGING CONFIGURATION
    LogName = f"{TrainConfig['LogName']}"
    
    ## SELECT ENVIRONMENT
    if TrainConfig['ENV_Type'] == "SAR_2D_Env":
        env = SAR_2D_Env
    elif TrainConfig['ENV_Type'] == "SAR_Sim_DeepRL":
        env = SAR_Sim_DeepRL

    ## SET UP TRAINING CONDITIONS FROM CONFIG
    env_kwargs = TrainConfig['ENV_KWARGS']
    POLICY_KWARGS = {
        "gamma": 0.999,
        "learning_rate": 0.002,
        "ent_coef": "auto_0.005",
        "target_entropy": -2,
        "batch_size": 256,
        "buffer_size": 200000,
        "net_arch": {
            "pi": [10, 10, 10],
            "qf": [64, 64, 64]
        }
    }

    ## CREATE RL MANAGER
    RL_Manager = RL_Training_Manager(env,LOG_DIR,LogName,env_kwargs=env_kwargs,S3_Upload=False)
    
    ## CREATE MODEL AND TRAIN
    RL_Manager.create_model(POLICY_KWARGS)
    RL_Manager.load_model(
        Log_name=PreTrainConfig['LogName'],
        t_step=PreTrainConfig['t_step_optim'],
        Params_only=True,
        load_replay_buffer=False
    )
    # RL_Manager.sweep_policy(V_mag_Step=1.0,V_angle_Step=10,n=1)
    RL_Manager.train_model(reset_timesteps=False,total_timesteps=TrainConfig['t_step_limit'])



    
