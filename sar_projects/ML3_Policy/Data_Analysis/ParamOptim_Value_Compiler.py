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

## ADD SAR_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(0,BASE_PATH)
from crazyflie_logging.data_analysis.Data_Analysis import DataFile

Initials = "NL"
dataPath = f"{BASE_PATH}/crazyflie_logging/local_logs/NL_NDR_Data"
compiledPath = f"{BASE_PATH}/crazyflie_projects/ML3_Policy/Data_Logs"
compiledName = f"{Initials}_LR_Trials_NDR.csv"

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

        "Vel_Rot","Phi_Rot",

        "Tau_Rot_mean",
        "OFy_Rot_mean",
        "D_ceil_Rot_mean",
        "My_mean",

        "Vz_Rot_mean",
        "Vx_Rot_mean",

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


        Vz_Rot_mean,vz_Rot_std,_ = trial.grab_trial_data(trial.grab_Rot_state,'vz')
        Vx_Rot_mean,vx_Rot_std,_ = trial.grab_trial_data(trial.grab_Rot_state,'vx')
        Tau_Rot_mean,Tau_Rot_std,_ = trial.grab_trial_data(trial.grab_Rot_state,'Tau')
        OFy_Rot_mean,OFy_Rot_std,_ = trial.grab_trial_data(trial.grab_Rot_state,'OF_y')
        D_ceil_Rot_mean,D_ceil_Rot_std,_ = trial.grab_trial_data(trial.grab_Rot_state,'d_ceil')
        My_mean,My_std,_ = trial.grab_trial_data(trial.grab_Rot_state,'My')
        landing_rate_4leg,_,_,_ = trial.landing_rate()
        mu,sigma = trial.grab_finalPolicy()

        Vel_Rot = np.sqrt(Vx_Rot_mean**2 + Vz_Rot_mean**2)
        Phi_Rot = np.rad2deg(np.arctan2(Vz_Rot_mean,Vx_Rot_mean))


        ## WRITE FILE HEADER
        with open(filePath,'a') as file:
            writer = csv.writer(file,delimiter=',')
            writer.writerow([
                Vel_IC, Phi_IC, Trial_num,
                landing_rate_4leg,

                np.round(Vel_Rot,3),np.round(Phi_Rot,3),

                Tau_Rot_mean,
                OFy_Rot_mean,
                D_ceil_Rot_mean,
                My_mean,

                Vz_Rot_mean,
                Vx_Rot_mean,

                mu,sigma,
            ])

    except:
        print(f"[EXCEPTION] FileName: {fileName} Skipping File")
        continue

    end_time = time.time()
    

