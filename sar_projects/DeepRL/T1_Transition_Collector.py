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
        "Render": True,
    }

    log_name = "DeepRL_Policy_03-24--13:26:10"
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL" 
    model_dir = f"/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL/{log_name}/Models"

    ## LOAD EXPERT MODEL
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)

    for _ in range(10):
        RL_Manager.collect_manual_policy_transition(V_mag=2.5,V_angle=60,Plane_Angle=0)