## STANDARD IMPORTS
import os
from datetime import datetime

import rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))


## IMPORT ENVIRONMENTS
from crazyflie_projects.DeepRL.Policy_Training_DeepRL import Policy_Trainer_DeepRL
from crazyflie_projects.Leg_Design_Analysis.Envs.CrazyflieEnv_DeepRL_LDA import CrazyflieEnv_DeepRL_LDA


## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%m_%d-%H:%M")



if __name__ == '__main__':

    ## INITIATE ENVIRONMENT
    env = CrazyflieEnv_DeepRL_LDA(GZ_Timeout=True,Vel_range=[0.5,3.5],Phi_range=[30,90])
    log_dir = f"{BASE_PATH}/crazyflie_projects/Leg_Design_Analysis/TB_Logs/{env.env_name}"

    ## CREATE NEW DEEP RL MODEL 
    log_name = f"SAC--{current_time}--{env.modelInitials}"    
    PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    PolicyTrainer.create_model()
    PolicyTrainer.train_model()
    
    # # LOAD DEEP RL MODEL
    # log_name = "SAC--01_02-16:22--NL_0"
    # t_step_load = 10

    # PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    # PolicyTrainer.load_model(t_step_load)
    # PolicyTrainer.train_model()
