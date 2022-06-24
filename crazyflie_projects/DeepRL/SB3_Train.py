import os
from datetime import datetime
from stable_baselines3 import PPO,SAC
from stable_baselines3.common.callbacks import *
import gym


from Brake_Trigger_Env import Brake_Trigger_Env
from CF_Env import CF_Env
# from CF_Env2 import CF_Env2
from CF_Env3 import CF_Env3



## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H-%M")

## INITIATE ENVIRONMENT
env = CF_Env3()
env.reset()


## CREATE MODEL AND LOG DIRECTORY
models_dir = f"crazyflie_projects/DeepRL/models/{env.env_name}/PPO-{current_time}"
log_dir = "/tmp/logs"

checkpoint_callback = CheckpointCallback(save_freq=2000, save_path=models_dir,name_prefix=env.env_name)
# eval_callback = EvalCallback(env, best_model_save_path=models_dir,
#                              log_path=log_dir, eval_freq=100,
#                              deterministic=True, render=False)


model = PPO(
    "MlpPolicy",
    env,
    gamma=0.999,
    learning_rate=0.002,
    use_sde=False,
    sde_sample_freq=4,
    verbose=1,
    create_eval_env=True,
    tensorboard_log=log_dir
) 

model.learn(total_timesteps=300e3,callback=checkpoint_callback)


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