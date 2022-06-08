import gym
from stable_baselines3 import A2C,PPO
import os


models_dir = "crazyflie_projects/DeepRL/models/PPO2"
log_dir = "crazyflie_projects/DeepRL/logs"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(log_dir):
    os.makedirs(log_dir)

env = gym.make("LunarLander-v2")
env.reset()


## INTERACT WITH ENV AND TRAIN MODEL
model = PPO("MlpPolicy",env,verbose=1,tensorboard_log=log_dir) 

TIMESTEPS = 10_000
for i in range(1,30):
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name="PPO2")
    model.save(f"{models_dir}/{TIMESTEPS*i}")




episodes = 10

for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        env.render()
        obs, reward,done,info, = env.step(env.action_space.sample())

env.close()