import os
from datetime import datetime
from stable_baselines3 import A2C,PPO

from Env_Example import CustomEnv
from Tau_Coast_Env import Tau_Coast_Env


## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H_%M")

## CREATE MODEL AND LOG DIRECTORY
models_dir = f"crazyflie_projects/DeepRL/models/PPO-{current_time}"
log_dir = "crazyflie_projects/DeepRL/logs"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(log_dir):
    os.makedirs(log_dir)


## INITIATE SB3 ALGORITHM AND ENVIRONMENT
env = Tau_Coast_Env()
env.reset()
model = PPO("MlpPolicy",env,verbose=1,tensorboard_log=log_dir) 

## TRAIN MODEL AND SAVE MODEL EVERY N TIMESTEPS
TIMESTEPS = 10_000
for i in range(1,30):
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=f"PPO-{current_time}")
    model.save(f"{models_dir}/{TIMESTEPS*i/1000}.zip")


## RENDER TRAINED MODEL FOR N EPISODES
episodes = 10
for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        env.render()
        action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)

env.close()