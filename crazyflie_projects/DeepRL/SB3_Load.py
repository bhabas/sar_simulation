from stable_baselines3 import PPO
from Tau_Trigger_Env import Tau_Trigger_Env
from Brake_Val_Env import Brake_Val_Env
from Discrete_Pos_Env import Discrete_Pos_Env
from Cont_Pos_Env import Cont_Pos_Env
from Cont_Value_Pred_Env import Cont_Value_Pred_Env


import gym

# ## INITIATE ENVIRONMENT AND TRAINED MODEL5
# env = Brake_Val_Env()
# env.reset()

# env = gym.make("Pendulum-v1")
env = Cont_Value_Pred_Env()
env.reset()
env.env_name = "Pendulum"

## SELECT MODEL FROM DIRECTORY
models_dir = f"crazyflie_projects/DeepRL/models/{env.env_name}/PPO-18-07"
model_path = f"{models_dir}/150.zip"


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