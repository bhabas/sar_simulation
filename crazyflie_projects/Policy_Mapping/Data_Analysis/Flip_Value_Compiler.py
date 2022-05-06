"""
This script runs through all of the data logs in a folder and compile the data by their
converged/average landing parameters to extract the average flip values
"""

import numpy as np
import pandas as pd
import os
import time
import rospy

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(0,BASE_PATH)
from crazyflie_logging.data_analysis.Data_Analysis import DataFile


dataPath = f"{BASE_PATH}/crazyflie_logging/local_logs/NL_Raw_Trials/"
compiledPath = f"{BASE_PATH}/crazyflie_projects/Policy_Mapping/Data_Logs/NL_Raw"

df_list = []
num_files = len(os.listdir(dataPath))

## COMPILING UPDATES
start_time = time.time()
end_time = time.time()
time_delta = 20 # Analysis time per file [s]
alpha = 0.15 # Filter parameter to smooth out time estimate

## ITER OVER ALL FILES IN DIR
for ii,fileName in enumerate(os.listdir(dataPath)): # Iter over all files in dir


    ## ESTIMATE ANALYSIS TIME PER FILE
    diff = end_time - start_time
    time_delta = alpha*diff + (1-alpha)*(time_delta)
    start_time = time.time()

    

    try: ## TRY OPENING FILE

        ## RECORD DATA FROM LOG FILE    
        print(f"Current File: {fileName} \t Index: {ii}/{num_files-1} \t Percentage: {100*ii/num_files:.2f}% \t Minutes to Completion: {time_delta/60*(num_files-ii):.1f}")

        trial = DataFile(dataPath,fileName,dataType='EXP')
        
        ## TRIAL CONDITIONS
        vel_IC,phi_IC = trial.grab_vel_IC_2D_angle()
        trial_num = int(fileName[-10:-8])


        vz_flip_mean,vz_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,'vz')
        vx_flip_mean,vx_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,'vx')
        Tau_flip_mean,Tau_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,'Tau')
        OFy_flip_mean,OFy_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,'OF_y')
        D_ceil_flip_mean,D_ceil_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,'d_ceil')
        My_mean,My_std,_ = trial.grab_trial_data(trial.grab_flip_state,'My')
        landing_rate_4leg,_,_,_ = trial.landing_rate()




        df_list.append((
            vel_IC, phi_IC, trial_num,
            landing_rate_4leg,
            vz_flip_mean,vz_flip_std,
            vx_flip_mean,vx_flip_std,
            Tau_flip_mean,Tau_flip_std,
            OFy_flip_mean,OFy_flip_std,
            D_ceil_flip_mean,D_ceil_flip_std,
            My_mean,My_std,
        ))




    except:
        print(f"[EXCEPTION] FileName: {fileName} Skipping File")
        continue

    end_time = time.time()
    
master_df = pd.DataFrame(df_list,columns=(
    "vel_IC", "phi_IC", "trial_num",
    "LR_4leg",
    "vz_flip_mean","vz_flip_std",
    "vx_flip_mean","vx_flip_std",
    "Tau_flip_mean","Tau_flip_std",
    "OFy_flip_mean","OFy_flip_std",
    "D_ceil_flip_mean","D_ceil_flip_std",
    "My_mean","My_std"
))
master_df = master_df.round(4)
master_df[['vel_IC','phi_IC','trial_num']] = master_df[['vel_IC','phi_IC','trial_num']].round(2)
master_df.sort_values(['vel_IC','phi_IC','trial_num'],ascending=[1,1,1],inplace=True)
master_df.to_csv(f'{compiledPath}/NL_LR_Trials_Raw.csv',index=False,mode='w',header=True)


