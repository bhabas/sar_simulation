import numpy as np
import pandas as pd
import os
import time
import warnings

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(0,BASE_PATH)
from crazyflie_logging.data_analysis.Data_Analysis import DataFile
dataPath = f"{BASE_PATH}/crazyflie_logging/local_logs/NL_Raw_Trials/"
compiledPath = f"{BASE_PATH}/crazyflie_projects/Policy_Mapping/Data_Logs"
df_list = []
num_files = len(os.listdir(dataPath))

start_time = time.time()
end_time = time.time()
time_delta = 20 # Analysis time per file [s]
alpha = 0.15 # Filter parameter to smooth out time estimate

file_index = 0

df = pd.DataFrame(columns=(
    'k_ep','k_run',
    'vel_IC','phi_IC','trial_num',
    'success_flag','num_contacts','body_contact',
    'vx','vz',
    'Tau','d_ceil','OF_y',
    'My_d'))


## ITER OVER ALL FILES IN DIR
for ii,fileName in enumerate(os.listdir(dataPath)): # Iter over all files in dir


    ## ESTIMATE ANALYSIS TIME PER FILE
    diff = end_time - start_time
    time_delta = alpha*diff + (1-alpha)*(time_delta)
    start_time = time.time()

    if ii%75 == 0:

        file_index += 1
        df.to_csv(f'{compiledPath}/test_file_{file_index}.csv',index=False)

    try: ## TRY OPENING FILE

        ## RECORD DATA FROM LOG FILE    
        print(f"Current File: {fileName} \t Index: {ii}/{num_files-1} \t Percentage: {100*ii/num_files:.2f}% \t Minutes to Completion: {time_delta/60*(num_files-ii):.1f}")

        trial = DataFile(dataPath,fileName,dataType='SIM')
        
        ## TRIAL CONDITIONS
        vel_IC,phi_IC = trial.grab_vel_IC_2D_angle()
        vx,_,vz = trial.grab_vel_IC()
        trial_num = int(fileName[-10:-8])

        for k_ep,k_run in zip(trial.k_df['k_ep'],trial.k_df['k_run']):

            try: ## TRY READING VALUES FROM FILE
            
                _,My_d = trial.grab_policy(k_ep,k_run)
                vx,vz,Tau,d_ceil,OF_y = trial.grab_flip_state(k_ep,k_run,['vx','vz','Tau','d_ceil','OF_y'])
                num_contacts,_,body_contact  = trial.landing_conditions(k_ep,k_run)

                if num_contacts >= 3 and body_contact == False:
                    success_flag = True
                else:
                    success_flag = False

                df_list.append((
                    k_ep,k_run,
                    vel_IC, phi_IC, trial_num,
                    success_flag,num_contacts,body_contact,
                    vx,vz,
                    Tau,d_ceil,OF_y,
                    My_d,
                ))
            except:
                print(f"[EXCEPTION] FileName: {fileName} K_ep: {k_ep} K_run: {k_run} | Skipping Line")

    except:
        print(f"[EXCEPTION] FileName: {fileName} Skipping File")
        continue


    
    master_df = pd.DataFrame(df_list).to_csv(f'{compiledPath}/test_file_{file_index}.csv',index=False,mode='a',header=False)
    end_time = time.time()
    




