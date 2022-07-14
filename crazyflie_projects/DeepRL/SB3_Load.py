from datetime import datetime
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *

from CF_Env_2D import CF_Env_2D
from CF_Env_2D_dTau import CF_Env_2D_dTau

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H-%M")

## INITIATE ENVIRONMENT
env = CF_Env_2D()


## CREATE MODEL AND LOG DIRECTORY
log_dir = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/logs/{env.env_name}"
log_name = f"SAC-15-23_0"
model_path = os.path.join(log_dir,log_name,f"models/{130}000_steps.zip")
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