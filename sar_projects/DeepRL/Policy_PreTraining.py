## STANDARD IMPORTS
import os
from datetime import datetime
import numpy as np
import rospkg

from RL_Manager import RL_Training_Manager
import imitation
import imitation.data.rollout as rollout


## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))

## COLLECT CURRENT TIME
current_datetime = datetime.now()
current_time = current_datetime.strftime("%m_%d-%H:%M")


if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
    from Envs.SAR_2D_DeepRL import SAR_2D_Env


    # ================================================================= ##

    # Define the environment parameters
    env_kwargs = {
        "Ang_Acc_range": [-100, 100],
        "V_mag_range": [2.5,2.5],
        "V_angle_range": [60,60],
        "Plane_Angle_range": [0,0],
        "Render": True,
        "GZ_Timeout": False,
    }


    log_name = "DeepRL_Policy_03-24--08:16:59"
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL" 
    model_dir = f"/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL/{log_name}/Models"

    ## LOAD EXPERT MODEL
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)
    RL_Manager.load_model(model_dir,t_step=22000)

    ## COLLECT EXPERT TRANSITIONS
    rollouts = rollout.rollout(
        RL_Manager.model,
        RL_Manager.vec_env,
        sample_until=rollout.make_sample_until(min_episodes=10),
        rng=np.random.default_rng(),
        exclude_infos=True,
        unwrap=False,
    )
    transitions = rollout.flatten_trajectories(rollouts)

    from imitation.algorithms import bc
    bc_trainer = bc.BC(
        observation_space=RL_Manager.env.observation_space,
        action_space=RL_Manager.env.action_space,
        demonstrations=transitions,
        rng=np.random.default_rng(),
    )

    bc_trainer.train(n_epochs=250)

    from stable_baselines3.common.evaluation import evaluate_policy
    reward_after_training, _ = evaluate_policy(bc_trainer.policy, RL_Manager.env, 10)
    print(f"Reward after training: {reward_after_training}")


    





