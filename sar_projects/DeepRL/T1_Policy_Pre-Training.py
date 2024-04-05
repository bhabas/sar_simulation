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
parser.add_argument('--config', help='Path to configuration file', required=True)
parser.add_argument('--S3_Upload', help='Upload to S3', default=False, type=bool)
args = parser.parse_args()

## LOAD CONFIGURATION
with open(args.config, 'r') as config_file:
    config = json.load(config_file)


## UPDATE SAR TYPE AND SAR CONFIG IN BASE SETTINGS FILE
Base_Settings_yaml = f"{BASE_PATH}/sar_config/Base_Settings.yaml"
with open(Base_Settings_yaml, 'r') as file:
    lines = file.readlines()

for i, line in enumerate(lines):
    if line.strip().startswith('SAR_Type:'):
        lines[i] = f"  SAR_Type: '{config['SAR_SETTINGS']['SAR_Type']}'\n"
    elif line.strip().startswith('SAR_Config:'):
        lines[i] = f"  SAR_Config: '{config['SAR_SETTINGS']['SAR_Config']}'\n"

with open(Base_Settings_yaml, 'w') as file:
    file.writelines(lines)


if __name__ == '__main__':

    ## LOGGING CONFIGURATION
    LogName = f"{config['LogName']}"

    POLICY_KWARGS = {
        "gamma": 0.999,
        "learning_rate": 0.002,
        "ent_coef": "auto_0.05",
        "target_entropy": -2,
        "batch_size": 256,
        "buffer_size": 200000,
        "net_arch": {
            "pi": [10, 10, 10],
            "qf": [64, 64, 64]
        }
    }
    
    ## SELECT ENVIRONMENT
    if config['ENV_Type'] == "SAR_2D_Env":
        env = SAR_2D_Env
    elif config['ENV_Type'] == "SAR_Sim_DeepRL":
        env = SAR_Sim_DeepRL

    ## SET UP TRAINING CONDITIONS FROM CONFIG
    env_kwargs = config['ENV_KWARGS']

    ## CREATE RL MANAGER
    RL_Manager = RL_Training_Manager(env,LOG_DIR,LogName,env_kwargs=env_kwargs,S3_Upload=False)
    
    ## CREATE MODEL AND TRAIN
    RL_Manager.create_model(POLICY_KWARGS)
    RL_Manager.train_model(reset_timesteps=False,total_timesteps=config['TRAINING']['t_step_limit'])
