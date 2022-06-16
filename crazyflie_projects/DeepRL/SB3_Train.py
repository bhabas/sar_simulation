import os
from datetime import datetime
from pickle import FALSE
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

## SELECT MODEL FROM DIRECTORY
models_dir = "crazyflie_projects/DeepRL/models/PPO-19_04"
model_path = f"{models_dir}/250.zip"

## INITIATE ENVIRONMENT AND TRAINED MODEL5
env = Tau_Coast_Env()
env.reset()
model = PPO.load(model_path,env=env)
# model = PPO("MlpPolicy",env,verbose=1,tensorboard_log=log_dir) 

## TRAIN MODEL AND SAVE MODEL EVERY N TIMESTEPS
TIMESTEPS = 10_000
for i in range(1,60):
    if i == 1:
        model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=True, tb_log_name=f"PPO-{current_time}")
    else:
        model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=f"PPO-{current_time}")
    model.save(f"{models_dir}/{TIMESTEPS*i//1000:d}.zip")


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