from datetime import datetime
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *

from Crazyflie_env import CrazyflieEnv

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H-%M")

## INITIATE ENVIRONMENT
env = CrazyflieEnv(gazeboTimeout=False)

## CREATE MODEL AND LOG DIRECTORY
BASEPATH = f"/home/bhabas/catkin_ws/src/crazyflie_simulation"
models_dir = f"{BASEPATH}/crazyflie_projects/DeepRL/models/{env.env_name}/SAC-{current_time}"
log_dir = "/home/bhabas/Downloads/logs"
checkpoint_callback = CheckpointCallback(save_freq=500, save_path=models_dir,name_prefix=env.env_name)

## SELECT MODEL FROM DIRECTORY
BASEPATH = f"/home/bhabas/catkin_ws/src/crazyflie_simulation"
models_dir = f"{BASEPATH}/crazyflie_projects/DeepRL/models/{'CF_Env_2D'}/SAC-{'CF_Env_2D'}-16-16"
model_path = f"{models_dir}/{'CF_Env_2D'}_{45000}_steps.zip"
model = SAC.load(model_path,env=env)

model.learn(
    total_timesteps=600e3,
    tb_log_name=f"SAC-{env.env_name}-{current_time}",
    callback=checkpoint_callback
)
