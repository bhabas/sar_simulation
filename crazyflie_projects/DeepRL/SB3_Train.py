import os
from datetime import datetime
from stable_baselines3 import PPO,SAC
from stable_baselines3.common.callbacks import CheckpointCallback
import gym


from Brake_Trigger_Env import Brake_Trigger_Env

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H-%M")

## INITIATE ENVIRONMENT
env = Brake_Trigger_Env()
# env = Cont_Value_Pred_Env()
# env = gym.make("Pendulum-v1")
# env.env_name = "Pendulum"
env.reset()

## CREATE MODEL AND LOG DIRECTORY
models_dir = f"crazyflie_projects/DeepRL/models/{env.env_name}/SAC-{current_time}"
log_dir = "/tmp/logs"

if not os.path.exists(log_dir):
    os.makedirs(log_dir)

checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=models_dir,name_prefix=env.env_name)

model = SAC(
    "MlpPolicy",
    env,
    gamma=0.99,
    learning_rate=0.001,
    use_sde=False,
    sde_sample_freq=4,
    verbose=1,
    create_eval_env=True,
    tensorboard_log=log_dir
) 

model.learn(30e3, callback=checkpoint_callback)


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