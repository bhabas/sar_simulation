import os,fnmatch
import numpy as np
import pandas as pd
os.system("clear")

## CREATE LINK TO DATAPATH MODULE
import sys
sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility')
from data_analysis import DataFile


dataPath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/"

## GENERATE INITIAL LIST VARIABLES
test_list = []
df_list = []


## DEFINE DATA RANGE TO ITERATE OVER
vz_array = np.arange(4.0,1.25,-0.25)    # Limits: [1.5,3.5]
vx_array = np.arange(0,3.0,0.25)        # Limits: [0,3.0]

## GENERATE TEST ARRAY
for vz_d in vz_array:      
    for vx_d in vx_array:   
        test_list.append([vz_d,vx_d])
test_arr = np.asarray(test_list)





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

            alpha_mu,alpha_sigma,mu_ini,sigma_ini = trial.grab_RLPararms()
            

            policy,sigma = trial.grab_finalPolicy()
            RREV_trigger,G1 = policy
            RREV_sig,G1_sig = sigma

            My_d = trial.grab_My_d_trial()
            impact_eul = trial.grab_impact_eul_trial('eul_y')


            df_list.append((
                vz_d,vx_d,trial_num,landing_rate,
                RREV_trigger,G1,
                RREV_sig,G1_sig,
                My_d,impact_eul,
                alpha_mu,alpha_sigma,
                mu_ini,sigma_ini,
                ))


            
master_df = pd.DataFrame(df_list,columns=(
    'vz_d','vx_d','trial_num','landing_rate',
    'RREV_trigger','G1',
    'RREV_sig','G1_sig',
    'My_d','impact_eul',
    'alpha_mu','alpha_sigma',
    'mu_ini','sigma_ini',
))
print(master_df)

master_df.to_csv('2-Term_Policy_Summary.csv',index=False)
