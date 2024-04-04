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


## ARGUMENT PARSER
parser = argparse.ArgumentParser(description='Policy Pre-Training Script')
parser.add_argument('--config', help='Path to configuration file', required=True)
parser.add_argument('--S3_Upload', help='Upload to S3', default=False, type=bool)
args = parser.parse_args()

# Load configuration from the specified file
with open(args.config, 'r') as config_file:
    config = json.load(config_file)

# config = "sar_projects/DeepRL/configs/config.yaml"
# with open(config, 'r') as config_file:
#     config = json.load(config_file)


if __name__ == '__main__':

    ## LOGGING CONFIGURATION
    LogDir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 
    current_time = datetime.now().strftime("%m-%d--%H:%M:%S")
    LogName = f"{config['TRAINING']['LogName']}_{current_time}"
    # ================================================================= ##
    
    ## SELECT ENVIRONMENT
    if config['TRAINING']['ENV_Type'] == "SAR_2D_Env":
        env = SAR_2D_Env
    elif config['TRAINING']['ENV_Type'] == "SAR_Sim_DeepRL":
        env = SAR_Sim_DeepRL

    ## SET UP TRAINING CONDITIONS FROM CONFIG
    env_kwargs = config['TRAINING']['ENV_KWARGS']
    policy_kwargs = config['TRAINING']['POLICY_KWARGS']

    ## CREATE RL MANAGER
    RL_Manager = RL_Training_Manager(env,LogDir,LogName,env_kwargs=env_kwargs,S3_Upload=False)

    ## CHECK SAR CONFIGURATION
    if config['SAR_SETTINGS']['SAR_Type'] != RL_Manager.env.SAR_Type:
        raise ValueError(f"Environment type mismatch: {config['SAR_SETTINGS']['SAR_Type']} vs {RL_Manager.env.SAR_Type}")
    
    elif config['SAR_SETTINGS']['SAR_Config'] != RL_Manager.env.SAR_Config:
        raise ValueError(f"Environment config mismatch: {config['SAR_SETTINGS']['SAR_Config']} vs {RL_Manager.env.SAR_Config}")

    
    ## CREATE MODEL AND TRAIN
    RL_Manager.create_model(policy_kwargs)
    RL_Manager.train_model(reset_timesteps=False,total_timesteps=config['TRAINING']['t_step_limit'])
