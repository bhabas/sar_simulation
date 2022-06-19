from stable_baselines3 import PPO
from Tau_Trigger_Env import Tau_Trigger_Env
from Brake_Val_Env import Brake_Val_Env
import gym

# ## INITIATE ENVIRONMENT AND TRAINED MODEL5
# env = Brake_Val_Env()
# env.reset()

env = gym.make("Pendulum-v1")
env.reset()
env.env_name = "Pendulum"

## SELECT MODEL FROM DIRECTORY
models_dir = f"crazyflie_projects/DeepRL/models/{env.env_name}/PPO-08-24"
model_path = f"{models_dir}/120.zip"


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