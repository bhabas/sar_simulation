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
parser.add_argument('--GroupName',      help='Log group name', default='')
parser.add_argument('--TrainConfig',    help='Path to training config file', required=True)
parser.add_argument('--t_step_load',    help='Time step to load model', default=None, type=int)
parser.add_argument('--S3_Upload',      help='Upload to S3', default=False, type=bool)
args = parser.parse_args()

## LOAD TRAINING MODEL CONFIGURATION
with open(args.TrainConfig, 'r') as TrainConfig_File:
    TrainConfig = json.load(TrainConfig_File)

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
    LogName = TrainConfig['LogName']
    PlaneAngle = TrainConfig['ENV_KWARGS']['Plane_Angle_range'][0]
    if args.t_step_load == None:
        t_step_load = TrainConfig['t_step_optim']
    else:
        t_step_load = args.t_step_load

    
    ## SELECT ENVIRONMENT
    if TrainConfig['ENV_Type'] == "SAR_2D_Env":
        env = SAR_2D_Env
    elif TrainConfig['ENV_Type'] == "SAR_Sim_DeepRL":
        env = SAR_Sim_DeepRL

    ## SET UP TRAINING CONDITIONS FROM CONFIG
    env_kwargs = TrainConfig['ENV_KWARGS']
    env_kwargs['V_mag_range'] = [3.5,5.0]

    ## CREATE RL MANAGER
    RL_Manager = RL_Training_Manager(env,args.GroupName,TrainConfig['LogName'],env_kwargs=env_kwargs,S3_Upload=args.S3_Upload)
    
    ## CREATE MODEL AND TRAIN
    RL_Manager.create_model()
    RL_Manager.load_model(
        GroupName=args.GroupName,
        LogName=TrainConfig['LogName'],
        t_step_load=t_step_load,
        Params_only=True,
    )

    RL_Manager.env.Render_Flag = True
    RL_Manager.env.Fine_Tuning_Flag = False
    RL_Manager.sweep_policy(Plane_Angle_Step=45,
        V_mag_Step=0.5,
        V_angle_Step=5,
        n=1)
## ADJUST HORIZONTAL CUTOFF AND REEVALUATE PERFORMANCE