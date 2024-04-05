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



# ## ARGUMENT PARSER
# parser = argparse.ArgumentParser(description='Policy Pre-Training Script')
# parser.add_argument('--config', help='Path to configuration file', required=True)
# parser.add_argument('--pretrained_model', help='Path to pretrained model', required=True)
# parser.add_argument('--S3_Upload', help='Upload to S3', default=False, type=bool)
# args = parser.parse_args()

# ## LOAD TRAINING MODEL CONFIGURATION
# with open(args.config, 'r') as config_file:
#     config = json.load(config_file)

# ## LOAD PRETRAINED MODEL CONFIGURATION
# with open(args.pretrained_model, 'r') as pretrained_model_file:
#     pretrained_model = json.load(pretrained_model_file)

config_path = "/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/Config_Files/SOV5_A45_L250_90deg_S2D.json"
pretrained_path = "/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/Config_Files/SOV5_A45_L150_90deg_S2D.json"

## LOAD TRAINING MODEL CONFIGURATION
with open(config_path, 'r') as config_file:
    config = json.load(config_file)

## LOAD PRETRAINED MODEL CONFIGURATION
with open(pretrained_path, 'r') as pretrained_model_file:
    pretrained_model = json.load(pretrained_model_file)


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
    
    ## SELECT ENVIRONMENT
    if config['ENV_Type'] == "SAR_2D_Env":
        env = SAR_2D_Env
    elif config['ENV_Type'] == "SAR_Sim_DeepRL":
        env = SAR_Sim_DeepRL

    ## SET UP TRAINING CONDITIONS FROM CONFIG
    env_kwargs = config['TRAINING']['ENV_KWARGS']
    policy_kwargs = config['TRAINING']['POLICY_KWARGS']

    ## CREATE RL MANAGER
    RL_Manager = RL_Training_Manager(env,LOG_DIR,LogName,env_kwargs=env_kwargs,S3_Upload=False)
    
    ## CREATE MODEL AND TRAIN
    RL_Manager.create_model(policy_kwargs)
    RL_Manager.load_model(
        Log_name=pretrained_model['LogName'],
        t_step=pretrained_model['TESTING']['t_step_optim'],
        Params_only=True,
        load_replay_buffer=False
    )
    # RL_Manager.sweep_policy(V_mag_Step=1.0,V_angle_Step=10,n=1)
    RL_Manager.train_model(reset_timesteps=False,total_timesteps=config['TRAINING']['t_step_limit'])



    
