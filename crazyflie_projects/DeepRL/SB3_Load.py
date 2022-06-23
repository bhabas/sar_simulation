from stable_baselines3 import PPO,SAC
from Brake_Trigger_Env import Brake_Trigger_Env
from CF_Env import CF_Env
from CF_Env2 import CF_Env2




import gym

# ## INITIATE ENVIRONMENT AND TRAINED MODEL5
env = CF_Env2()
env.reset()

## SELECT MODEL FROM DIRECTORY
models_dir = f"crazyflie_projects/DeepRL/models/{env.env_name}/SAC-17-18"
model_path = f"{models_dir}/{env.env_name}_12000_steps.zip"


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