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
env = CF_Env_2D()


## CREATE MODEL AND LOG DIRECTORY
log_dir = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/logs/{env.env_name}"
log_name = f"SAC-10-37_0"
model_path = os.path.join(log_dir,log_name,f"models/{96}000_steps.zip")
model = SAC.load(model_path,env=env,device='cpu')


def custom_predict(obs):

    # CAP the standard deviation of the actor
    LOG_STD_MAX = 2
    LOG_STD_MIN = -20
    actor = model.policy.actor
    obs = th.tensor([obs])
    model.policy.actor.forward(obs)

    ## PASS OBS THROUGH NN
    latent_pi = actor.latent_pi(obs)
    mean_actions = actor.mu(latent_pi)
    log_std = actor.log_std(latent_pi)
    log_std = th.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)

    ## CONVERT MU/LOG_STD TO NORMAL DISTRIBUTION AND SAMPLE
    action_std = th.ones_like(mean_actions) * log_std.exp()
    action_mean = mean_actions
    # samples = th.normal(action_mean,action_std)
    # scaled_action = th.tanh(samples).detach().numpy()

    ## CENTRAL LIMIT THEOREM SAMPLE
    normal_sample = np.sum(np.random.uniform(size=(12,2)),axis=0)-6
    samples = normal_sample*action_std.detach().numpy()+action_mean.detach().numpy()
    scaled_action = np.tanh(samples)

    ## SQUISH SAMPLES TO [-1,1] RANGE AND RESCALE
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
        action = custom_predict(obs)[0]
        # action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)

env.close()