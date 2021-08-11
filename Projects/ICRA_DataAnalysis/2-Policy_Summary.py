import os,time,datetime
import pandas as pd
import warnings
import send2trash
import numpy as np
import csv


## CREATE LINK TO DATAPATH MODULE
import sys
os.system("clear")
sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/')
from data_analysis.Data_Analysis import DataFile



dataPath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/local_logs/ExtraNarrow-Long_2-Policy/"
df_list = []
num_files = len(os.listdir(dataPath))

run_avg = 20
start_time = time.time()
end_time = time.time()
alpha = 0.15


## ITER OVER ALL FILES IN DIR
for ii,fileName in enumerate(os.listdir(dataPath)): # Iter over all files in dir

    try:
        ## PROGRESS PRINTING (BASIC STUFF STARTING FOR TIME ESTIMATION)
        

        diff = end_time - start_time

        run_avg = alpha*diff + (1-alpha)*(run_avg)
        start_time = time.time()

        
            

            

        print(f"Current File: {fileName} \t Index: {ii}/{num_files-1} \t Percentage: {100*ii/num_files:.2f}% \t Minutes to Completion: {run_avg/60*(num_files-ii):.1f}")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)

            trial = DataFile(dataPath,fileName,dataType='SIM')
            
            ## TRIAL CONDITIONS
            vel_IC,phi_IC = trial.grab_vel_IC_2D_angle()
            vx,_,vz = trial.grab_vel_IC()
            trial_num = trial.trialNum

            ## RL PARAMETERS
            alpha_mu,alpha_sigma,mu_0,sigma_0 = trial.grab_RLParams()
            policy,sigma = trial.grab_finalPolicy()
            RREV_threshold,My_d = policy
            RREV_sig,My_d_sig = sigma

            ## LANDING RATES
            landing_rate_4_leg,landing_rate_2_leg,_,_ = trial.landing_rate(N=3)


            ## FLIP DATA
            RREV_flip_mean,RREV_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,stateName='RREV')
            dRREV_flip_mean,dRREV_flip_std,_ = trial.grab_trial_data(trial.grab_flip_dRREV)
            OF_y_flip_mean,OF_y_flip_std,_ = trial.grab_trial_data(trial.grab_flip_state,stateName='OF_y')
            flip_height_mean,flip_height_std,_ = trial.grab_trial_data(trial.grab_flip_state,stateName='z')
            flip_d_mean = 2.1 - flip_height_mean
            flip_vz_mean,flip_vz_std,_ = trial.grab_trial_data(trial.grab_flip_state,stateName='vz')

            ## IMPACT DATA
            impact_force_x_mean,impact_force_x_std,_ = trial.grab_trial_data(trial.grab_impact_force,'x')
            impact_force_z_mean,impact_force_z_std,_ = trial.grab_trial_data(trial.grab_impact_force,'z')
            impact_vx_mean,impact_vx_std,_ = trial.grab_trial_data(trial.grab_impact_state,stateName='vx')
            impact_vz_mean,impact_vz_std,_ = trial.grab_trial_data(trial.grab_impact_state,stateName='vz')
            t_delta_mean,t_delta_std,_ = trial.grab_trial_data(trial.trigger2impact)
            
            impact_eul_mean,impact_eul_std,_ = trial.grab_impact_eul_trial(eul_type='eul_y')

            ## GRAB FULL IMPACT/SUCCESS PICTURE
            # _,_,impact_eul_arr = trial.grab_impact_eul_trial(eul_type='eul_y',landing_cutoff=0,N=1)
            # _,_,impact_vx_arr = trial.grab_trial_data(trial.grab_impact_state,stateName='vx',landing_cutoff=0,N=1)
            # _,_,impact_vz_arr = trial.grab_trial_data(trial.grab_impact_state,stateName='vz',landing_cutoff=0,N=1)



        df_list.append((
            vel_IC, phi_IC, trial_num,
            vx,vz,
            mu_0,sigma_0,
            RREV_threshold,RREV_sig,
            My_d,My_d_sig,
            landing_rate_4_leg,landing_rate_2_leg,

            RREV_flip_mean,RREV_flip_std,
            dRREV_flip_mean,dRREV_flip_std,
            OF_y_flip_mean,OF_y_flip_std,
            flip_height_mean,flip_height_std,
            flip_d_mean,
            flip_vz_mean,flip_vz_std,

            impact_force_x_mean,impact_force_x_std,
            impact_force_z_mean,impact_force_z_std,
            impact_vx_mean,impact_vx_std,
            impact_vz_mean,impact_vz_std,
            impact_eul_mean,impact_eul_std,
            t_delta_mean,t_delta_std,
        ))

        end_time = time.time()
        # if ii == 1:
        #     break

    except:
        # send2trash.send2trash(dataPath+fileName)
        end_time = time.time()
        print(f"Trashing file {fileName}")
        # pass

    


print()

master_df = pd.DataFrame(df_list,columns=(
    'vel_IC','phi_IC','trial_num',
    'vx','vz',
    'mu_0','sigma_0',
    'RREV_threshold','RREV_sig',
    'My_d','My_d_sig',
    'landing_rate_4_leg','landing_rate_2_leg',

    'RREV_flip_mean','RREV_flip_std',
    'dRREV_flip_mean','dRREV_flip_std',
    'OF_y_flip_mean','OF_y_flip_std',
    'flip_height_mean','flip_height_std',
    'flip_d_mean',
    'flip_vz_mean','flip_vz_std',

    'impact_force_x_mean','impact_force_x_std',
    'impact_force_z_mean','impact_force_z_std',
    'impact_vx_mean','impact_vx_std',
    'impact_vz_mean','impact_vz_std',
    'impact_eul_mean','impact_eul_std',
    't_delta_mean','t_delta_std',
))
print(master_df)
master_df = master_df.round(4)
master_df.sort_values(['vel_IC','phi_IC','trial_num'],ascending=[1,1,1],inplace=True)

master_df.to_csv('Narrow-Long_2-Policy_Summary.csv',index=False,quoting=csv.QUOTE_MINIMAL)