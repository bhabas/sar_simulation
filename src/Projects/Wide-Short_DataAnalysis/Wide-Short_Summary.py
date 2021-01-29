import os,fnmatch
import numpy as np
import pandas as pd
os.system("clear")

## CREATE LINK TO DATAPATH MODULE
import sys
sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility')
from data_analysis import DataFile

test_list = []
for vz_d in np.arange(4.0,1.25,-0.25): # [1.5,3.5]
    for vx_d in np.arange(0,3.0,0.25):   # [0,3.0]
        test_list.append([vz_d,vx_d])
test_arr = np.asarray(test_list)



dataPath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/temp_log/"

df_list = []
## ITER OVER ALL TEST COMBINATIONS
for vz_d,vx_d in test_arr: 

    ## ITER OVER ALL FILES IN DIR
    for fileName in os.listdir(dataPath): # Iter over all files in dir
        
        ## IF FILE MATCHES DESIRED IC
        if fnmatch.fnmatch(fileName,f"*Vz_{vz_d:.2f}--Vx_{vx_d:.2f}*.csv"):
            
            
            trial = DataFile(dataPath,fileName)
            print(fileName)
            
            trial_num = fileName[-5]
            landing_rate = trial.landing_rate()


            policy,sigma = trial.grab_finalPolicy()
            RREV_trigger,G1,G2 = policy
            RREV_sig,G1_sig,G2_sig = sigma

            My_d = trial.grab_M_d_trial()[1]
            impact_eul = trial.grab_impact_eul_trial('eul_y')


            df_list.append((
                vz_d,vx_d,trial_num,landing_rate,
                RREV_trigger,G1,G2,
                RREV_sig,G1_sig,G2_sig,
                My_d,impact_eul
                ))


            
master_df = pd.DataFrame(df_list,columns=(
    'vz_d','vx_d','trial_num','landing_rate',
    'RREV_trigger','G1','G2',
    'RREV_sig','G1_sig','G2_sig',
    'My_d','impact_eul'
))
print(master_df)