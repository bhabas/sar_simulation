from datetime import datetime
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
import torch as th
import numpy as np
from CF_Env_2D import CF_Env_2D
from CF_Env_2D_dTau import CF_Env_2D_dTau
from CF_Env_2D_Simple import CF_Env_2D_Simple

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H-%M")

## INITIATE ENVIRONMENT
env = CF_Env_2D_Simple()

# obs = tensor([[ 0.1654, -2.1964,  0.4661]])

## CREATE MODEL AND LOG DIRECTORY
log_dir = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/logs/{env.env_name}"
log_name = f"SAC-08-24_0"
model_path = os.path.join(log_dir,log_name,f"models/{24}000_steps.zip")
model = SAC.load(model_path,env=env,device='cpu')



def custom_predict(obs):

    # CAP the standard deviation of the actor
    LOG_STD_MAX = 2
    LOG_STD_MIN = -20
    actor = model.policy.actor
    # obs = th.tensor([[ 0.1654, -2.1964,  0.4661]])
    
    # obs = th.tensor([[ 0.15535472, -2.337496  ,  0.43795708]])

    obs = th.tensor([obs])
    latent_pi = actor.latent_pi(obs)
    mean_actions = actor.mu(latent_pi)
    log_std = actor.log_std(latent_pi)
    log_std = th.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)
    action_std = th.ones_like(mean_actions) * log_std.exp()
    dist = np.random.normal(mean_actions.item(),action_std.item(),1)
    scaled_action = np.tanh(dist)

    model.policy.actor.forward(obs)

    low, high = env.action_space.low, env.action_space.high
    return low + (0.5 * (scaled_action + 1.0) * (high - low))


# ## RENDER TRAINED MODEL FOR N EPISODES-
episodes = 50
env.RENDER = True
for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        env.render()
        action = custom_predict(obs)
        # action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)

env.close()