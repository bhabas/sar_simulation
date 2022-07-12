import os
from datetime import datetime
from stable_baselines3 import PPO,SAC
from stable_baselines3.common.callbacks import *
import gym


from CF_Env_2D import CF_Env_2D
from CF_Env_2D_dTau import CF_Env_2D_dTau



## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H-%M")

## INITIATE ENVIRONMENT
env = CF_Env_2D()
env.reset()


## CREATE MODEL AND LOG DIRECTORY
BASEPATH = f"/home/bhabas/catkin_ws/src/crazyflie_simulation"
models_dir = f"{BASEPATH}/crazyflie_projects/DeepRL/models/{env.env_name}/SAC-{env.env_name}-{current_time}"
log_dir = "/home/bhabas/Downloads/logs"




class TensorboardCallback(BaseCallback):
    """
    Custom callback for plotting additional values in tensorboard.
    """
    def __init__(self, verbose=0):
        super(TensorboardCallback, self).__init__(verbose)

    def _on_step(self) -> bool:
        return True

    def _on_rollout_end(self) -> None:
        self.logger.record('time/K_ep',self.training_env.envs[0].env.k_ep)
        return True

checkpoint_callback = CheckpointCallback(save_freq=5000, save_path=models_dir,name_prefix=env.env_name)
callback = CallbackList([checkpoint_callback,TensorboardCallback()])

model = SAC(
    "MlpPolicy",
    env,
    gamma=0.999,
    learning_rate=0.001,
    verbose=1,
    device='cpu',
    create_eval_env=True,
    tensorboard_log=log_dir
) 

model.learn(
    total_timesteps=1e6,
    tb_log_name=f"SAC-{env.env_name}-{current_time}",
    callback=callback
)


env.close()