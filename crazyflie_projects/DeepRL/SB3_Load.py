from stable_baselines3 import PPO,SAC
from CF_Env import CF_Env




import gym

# ## INITIATE ENVIRONMENT AND TRAINED MODEL5
env = CF_Env()
env.reset()

## SELECT MODEL FROM DIRECTORY
BASEPATH = f"/home/bhabas/catkin_ws/src/crazyflie_simulation"
models_dir = f"{BASEPATH}/crazyflie_projects/DeepRL/models/{env.env_name}/SAC-{env.env_name}-15-50"
model_path = f"{models_dir}/{env.env_name}_{40}000_steps.zip"


model = SAC.load(model_path,env=env)

## RENDER TRAINED MODEL FOR N EPISODES-
episodes = 50
env.RENDER = True
for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        env.render()
        action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)

env.close()