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
BASEPATH = f"/home/bhabas/catkin_ws/src/crazyflie_simulation"
models_dir = f"{BASEPATH}/crazyflie_projects/DeepRL/models/{env.env_name}/SAC-{current_time}"
log_dir = "/home/bhabas/Downloads/logs"

checkpoint_callback = CheckpointCallback(save_freq=5000, save_path=models_dir,name_prefix=env.env_name)


class TensorboardCallback(BaseCallback):
    """
    Custom callback for plotting additional values in tensorboard.
    """

    def __init__(self, verbose=0):
        super(TensorboardCallback, self).__init__(verbose)

    def _on_step(self) -> bool:
        # Log scalar value (here a random variable)
        # self.model.policy.action_dist.distribution.mean
        # self.model.policy.action_dist.distribution.scale
        return True

    def _on_rollout_end(self) -> None:
        # self.logger.record('rollout/Theta_Impact',self.training_env.envs[0].env.theta_impact)
        # self.logger.record('rollout/R1',self.training_env.envs[0].env.R1)
        # self.logger.record('rollout/R2',self.training_env.envs[0].env.R2)
        # self.logger.record('rollout/R3',self.training_env.envs[0].env.R3)
        self.logger.record('time/K_ep',self.training_env.envs[0].env.k_ep)


        return True

callback = CallbackList([checkpoint_callback,TensorboardCallback()])

model = SAC(
    "MlpPolicy",
    env,
    gamma=0.999,
    learning_rate=0.001,
    use_sde=False,
    sde_sample_freq=4,
    verbose=1,
    device='cpu',
    create_eval_env=True,
    tensorboard_log=log_dir
) 

model.learn(
    total_timesteps=100e3,
    tb_log_name=f"SAC-{current_time}",
    callback=callback
)


env.close()