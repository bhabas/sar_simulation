## STANDARD IMPORTS
import os
from datetime import datetime
import numpy as np
import rospkg

from RL_Manager import RL_Training_Manager



## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))

## COLLECT CURRENT TIME
current_datetime = datetime.now()
current_time = current_datetime.strftime("%m_%d-%H:%M")


if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
    from Envs.SAR_2D_DeepRL import SAR_2D_Env


    current_datetime = datetime.now()
    current_time = current_datetime.strftime("%m-%d--%H:%M:%S")
    log_name = f"DeepRL_Policy_{current_time}"
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL" 

    # ================================================================= ##

    # Define the environment parameters
    env_kwargs = {
        "Ang_Acc_range": [-100, 100],
        "V_mag_range": [2.5,2.5],
        "V_angle_range": [60,60],
        "Plane_Angle_range": [0,0],
        "Render": False,
        "GZ_Timeout": False,
    }


    log_name = "DeepRL_Policy_03-25--13:28:56"
    model_dir = f"/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL/{log_name}/Models"
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)
    RL_Manager.create_model(net_arch=[10,10,10])
    RL_Manager.load_transitions_from_csv()
    # RL_Manager.train_model(reset_timesteps=False)
    # batch = RL_Manager.model.replay_buffer.sample(256)

    for _ in range(1000):
        RL_Manager.model.train(1)


    


    # RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)
    # RL_Manager.load_model(model_dir,t_step=25000)
    # # RL_Manager.sweep_policy(Plane_Angle_range=[0,0,45],V_mag_range=[2.5,2.5,1.0],V_angle_range=[60,60,40],n=5)
    # your_trajectories = rollout.rollout(
    #     RL_Manager.model,
    #     RL_Manager.vec_env,
    #     sample_until=rollout.make_sample_until(min_episodes=5),
    #     rng=np.random.default_rng(),
    #     exclude_infos=True,
    #     unwrap=False,
    # )

    # print()


