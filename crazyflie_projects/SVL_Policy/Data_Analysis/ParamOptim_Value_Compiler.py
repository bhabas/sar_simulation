"""
This script runs through all of the data logs in a folder and for 
each file compiles the data down to the converged upon data 
"""

import numpy as np
import pandas as pd
import os
import time
import re
import csv

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(0,BASE_PATH)
from crazyflie_logging.data_analysis.Data_Analysis import DataFile


dataPath = f"{BASE_PATH}/crazyflie_logging/local_logs"
compiledPath = f"{BASE_PATH}/crazyflie_projects/SVL_Policy/Data_Logs"
compiledName = "NL_LR_Trials3.csv"

filePath = os.path.join(compiledPath,compiledName)

file_index = 0
num_files = len(os.listdir(dataPath))


## TIME ESTIMATION FILTER INITIALIZATION
start_time = time.time()
end_time = time.time()
time_delta = 20 # Estimated analysis time per file [s]
alpha = 0.15 # Filter parameter to smooth out time estimate


## WRITE FILE HEADER
with open(filePath,'w') as file:
    writer = csv.writer(file,delimiter=',')
    writer.writerow([
        "Vel_d", "Phi_d", "Trial_num",
        "LR_4Leg",

        "Vel_flip","Phi_flip",

        "Tau_flip_mean",
        "OFy_flip_mean",
        "D_ceil_flip_mean",
        "My_mean",

        "Vz_flip_mean",
        "Vx_flip_mean",

        "Mu","Sigma",
    ])

## ITER OVER ALL FILES IN DIR
for ii,fileName in enumerate(sorted(os.listdir(dataPath))): # Iter over all files in dir


    ## ESTIMATE ANALYSIS TIME PER FILE
    diff = end_time - start_time
    time_delta = alpha*diff + (1-alpha)*(time_delta)
    start_time = time.time()

    try: ## TRY OPENING FILE

        ## RECORD DATA FROM LOG FILE    
        print(f"Current File: {fileName} \t Index: {ii}/{num_files-1} \t Percentage: {100*ii/num_files:.2f}% \t Minutes to Completion: {time_delta/60*(num_files-ii):.1f}")

        trial = DataFile(dataPath,fileName,dataType='Sim')
        trial.n_rollouts = 6
        
        ## TRIAL CONDITIONS
        Vel_IC,Phi_IC = trial.grab_vel_IC_2D_angle()
        Trial_num = int(re.split("trial_",fileName)[1][:2])


        Vz_flip_mean,vz_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,'vz')
        Vx_flip_mean,vx_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,'vx')
        Tau_flip_mean,Tau_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,'Tau')
        OFy_flip_mean,OFy_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,'OF_y')
        D_ceil_flip_mean,D_ceil_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,'d_ceil')
        My_mean,My_std,_ = trial.grab_trial_data(trial.grab_flip_state,'My')
        landing_rate_4leg,_,_,_ = trial.landing_rate()
        mu,sigma = trial.grab_finalPolicy()

        Vel_flip = np.sqrt(Vx_flip_mean**2 + Vz_flip_mean**2)
        Phi_flip = np.rad2deg(np.arctan2(Vz_flip_mean,Vx_flip_mean))


        ## WRITE FILE HEADER
        with open(filePath,'a') as file:
            writer = csv.writer(file,delimiter=',')
            writer.writerow([
                Vel_IC, Phi_IC, Trial_num,
                landing_rate_4leg,

                np.round(Vel_flip,3),np.round(Phi_flip,3),

                Tau_flip_mean,
                OFy_flip_mean,
                D_ceil_flip_mean,
                My_mean,

                Vz_flip_mean,
                Vx_flip_mean,

                mu,sigma,
            ])

        # df_list.append((
        #     Vel_IC, Phi_IC, Trial_num,
        #     landing_rate_4leg,

        #     Vel_flip,Phi_flip,

        #     Tau_flip_mean,
        #     OFy_flip_mean,
        #     D_ceil_flip_mean,
        #     My_mean,

        #     Vz_flip_mean,
        #     Vx_flip_mean,

        #     mu,sigma,
        # ))




    except:
        print(f"[EXCEPTION] FileName: {fileName} Skipping File")
        continue

    end_time = time.time()
    


# # master_df = master_df.round(4)
# # master_df[['Vel_d','Phi_d','Trial_num']] = master_df[['Vel_d','Phi_d','Trial_num']].round(2)
# # master_df.sort_values(['Vel_d','Phi_d','Trial_num'],ascending=[1,1,1],inplace=True)
# # master_df.to_csv(f'{compiledPath}/{compiledName}',index=False,mode='w',header=True)



