import os
from datetime import datetime
from stable_baselines3 import PPO,SAC
from stable_baselines3.common.callbacks import *
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.sac.policies import MlpPolicy



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

eval_callback = EvalCallback(env, best_model_save_path=models_dir,
                             eval_freq=100,
                             deterministic=False, render=False,
                             callback_on_new_best=TensorboardCallback())
callback = CallbackList([eval_callback])

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
    total_timesteps=5000,
    tb_log_name=f"SAC-{env.env_name}-{current_time}",
    callback=callback
)

# model.save("Test_Model")
# model.save_replay_buffer("Test_Model_Replay_Buffer")
# model.policy.save("Test_Model_Policy")
# print(f"The loaded_model has {model.replay_buffer.size()} transitions in its buffer")
