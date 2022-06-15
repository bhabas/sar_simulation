import gym
from stable_baselines3 import A2C,PPO

from Env_Example import CustomEnv
import os

from datetime import datetime
now = datetime.now()
current_time = now.strftime("%H:%M")

models_dir = f"crazyflie_projects/DeepRL/models/PPO-{current_time}"
log_dir = "crazyflie_projects/DeepRL/logs"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(log_dir):
    os.makedirs(log_dir)

env = CustomEnv()
env.reset()


## INTERACT WITH ENV AND TRAIN MODEL
model = PPO("MlpPolicy",env,verbose=1,tensorboard_log=log_dir) 

TIMESTEPS = 10_000
for i in range(1,30):
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=f"PPO-{current_time}")
    model.save(f"{models_dir}/{TIMESTEPS*i}")




episodes = 10

for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        env.render()
        obs, reward,done,info, = env.step(env.action_space.sample())

env.close()