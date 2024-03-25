## STANDARD IMPORTS
import os
from datetime import datetime
import numpy as np
import rospkg

from RL_Manager import RL_Training_Manager
from imitation.algorithms import bc
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common import policies, torch_layers, utils, vec_env

from stable_baselines3.sac import policies as sac_policies


## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))

## COLLECT CURRENT TIME
current_datetime = datetime.now()
current_time = current_datetime.strftime("%m_%d-%H:%M")


if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
    from Envs.SAR_2D_DeepRL import SAR_2D_Env


    # Define the environment parameters
    env_kwargs = {
        "Ang_Acc_range": [-100, 100],
        "V_mag_range": [2.5,2.5],
        "V_angle_range": [60,60],
        "Plane_Angle_range": [0,0],
        "Render": True,
        "GZ_Timeout": False,
    }

    log_name = "DeepRL_Policy_03-24--13:26:10"
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL" 
    model_dir = f"/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL/{log_name}/Models"
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)

    # ================================================================= ##
    ## LOAD TRANSITIONS
    transitions = RL_Manager.load_transitions_from_csv()
    
    # RL_Manager.create_model()

    import torch as th
    from imitation.policies import base as policy_base


    policy = policies.ActorCriticPolicy(
                    observation_space=RL_Manager.env.observation_space,
                    action_space=RL_Manager.env.action_space,
                    lr_schedule=lambda _: th.finfo(th.float32).max,
                    net_arch = dict(pi=[10,10,10], qf=[256,256,256]), 
                    activation_fn=th.nn.LeakyReLU
                    )
    
    RL_Manager.create_model()
    


    ## PERFORM BEHAVIORAL CLONING TRAINING
    bc_trainer = bc.BC(
        observation_space=RL_Manager.env.observation_space,
        action_space=RL_Manager.env.action_space,
        demonstrations=transitions,
        device="cpu",
        policy=policy,
        rng=np.random.default_rng(),
    )
    trained_weights = bc_trainer.policy.state_dict()
    bc_trainer.train(n_epochs=1000)

    ## SAVE PRE-TRAINED MODEL
    # RL_Manager.save_model()
    # RL_Manager.sweep_policy(Plane_Angle_range=[0,0,45],V_mag_range=[2.5,2.5,5],V_angle_range=[60,60,10],n=5)

    # reward_after_training, _ = evaluate_policy(bc_trainer.policy, RL_Manager.env, 10)
    # print(f"Reward after training: {reward_after_training}")


    





