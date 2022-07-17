from datetime import datetime
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *

import torch as th
from CF_Env_2D import CF_Env_2D
from CF_Env_2D_dTau import CF_Env_2D_dTau

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H-%M")

## INITIATE ENVIRONMENT
env = CF_Env_2D()


## CREATE MODEL AND LOG DIRECTORY
log_dir = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/logs/{env.env_name}"
log_name = f"SAC-{15}-{33}_0"
model_path = os.path.join(log_dir,log_name,f"models/{10}000_steps.zip")
model = SAC.load(model_path,env=env,device='cpu')


obs = env.reset()
obs2 = th.Tensor([[0,0,0]])
# action,_ = model.predict(obs)

mu,log_std,_ = model.actor.get_action_dist_params(obs2)
