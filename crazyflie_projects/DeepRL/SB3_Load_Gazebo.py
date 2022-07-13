## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)

from crazyflie_env.src.Crazyflie_env import CrazyflieEnv

from stable_baselines3 import SAC

# ## INITIATE ENVIRONMENT AND TRAINED MODEL5
env = CrazyflieEnv()

## SELECT MODEL FROM DIRECTORY
BASEPATH = f"/home/bhabas/catkin_ws/src/crazyflie_simulation"
models_dir = f"{BASEPATH}/crazyflie_projects/DeepRL/models/{env.env_name}/SAC-22-54"
model_path = f"{models_dir}/{env.env_name}_{25500}_steps.zip"
model = SAC.load(model_path,env=env)


## RENDER TRAINED MODEL FOR N EPISODES-
episodes = 50
for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)
