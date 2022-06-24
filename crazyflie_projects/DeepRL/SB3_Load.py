from stable_baselines3 import PPO,SAC
from Brake_Trigger_Env import Brake_Trigger_Env
from CF_Env import CF_Env
from CF_Env2 import CF_Env2
from CF_Env3 import CF_Env3




import gym

# ## INITIATE ENVIRONMENT AND TRAINED MODEL5
env = CF_Env3()
env.reset()

## SELECT MODEL FROM DIRECTORY
models_dir = f"crazyflie_projects/DeepRL/models/CF_Env2/SAC-21-50"
model_path = f"{models_dir}/CF_Env2_100000_steps.zip"


model = SAC.load(model_path,env=env)

## RENDER TRAINED MODEL FOR N EPISODES
episodes = 25
env.RENDER = True
for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        env.render()
        action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)

env.close()