from stable_baselines3 import PPO
from Env_Example import CustomEnv
from Tau_Coast_Env import Tau_Coast_Env

## SELECT MODEL FROM DIRECTORY
models_dir = "crazyflie_projects/DeepRL/models/PPO-09_02"
model_path = f"{models_dir}/230.zip"

## INITIATE ENVIRONMENT AND TRAINED MODEL5
env = Tau_Coast_Env()
env.reset()
model = PPO.load(model_path,env=env)

## RENDER TRAINED MODEL FOR N EPISODES
episodes = 25
for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        env.render()
        action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)

env.close()