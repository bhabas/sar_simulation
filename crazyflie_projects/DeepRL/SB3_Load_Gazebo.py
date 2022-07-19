from datetime import datetime
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
import torch as th

from CF_Env_2D import CF_Env_2D
from CF_Env_2D_dTau import CF_Env_2D_dTau

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)
from crazyflie_env.src.Crazyflie_env import CrazyflieEnv

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H-%M")

## INITIATE ENVIRONMENT
env = CrazyflieEnv()


## CREATE MODEL AND LOG DIRECTORY
log_dir = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/logs/"
    
policy_kwargs = dict(activation_fn=th.nn.ReLU,
                     net_arch=[16, 16])
# model = SAC(
#     "MlpPolicy",
#     env=env,
#     gamma=0.999,
#     learning_rate=0.001,
#     policy_kwargs=policy_kwargs,
#     verbose=1,
#     device='cpu',
#     tensorboard_log=log_dir
# ) 
# model.set_parameters(
#     load_path_or_dict=f"{log_dir}/CF_Env_2D/SAC-15-23_0/models/135000_steps.zip",
#     device='cpu'
# )


model = SAC.load(
    path=f"{log_dir}/CF_Gazebo/SAC-22-20_0/models/{37}000_steps.zip",
    env=env,
    device='cpu'
)
model.load_replay_buffer(
    path=f"{log_dir}/CF_Gazebo/SAC-22-20_0/models/replay_buff.pkl")



## RENDER TRAINED MODEL FOR N EPISODES-
episodes = 50
for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)
