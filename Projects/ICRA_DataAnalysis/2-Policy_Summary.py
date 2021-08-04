import os,time,datetime
import pandas as pd
import send2trash


## CREATE LINK TO DATAPATH MODULE
import sys
os.system("clear")
sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/')
from data_analysis.Data_Analysis import DataFile



dataPath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/local_logs/Narrow-Wide_2-Policy/"
df_list = []
num_files = len(os.listdir(dataPath))

# run_avg = 0
# run_avg_prev = 0
# start_time = datetime.datetime.now()
# end_time = datetime.datetime.now()


## ITER OVER ALL FILES IN DIR
for ii,fileName in enumerate(os.listdir(dataPath)): # Iter over all files in dir

    # try:
#         ## PROGRESS PRINTING (BASIC STUFF STARTING FOR TIME ESTIMATION)
        

#         diff = end_time - start_time
#         alpha = 0.5
#         run_avg = alpha*run_avg + (1-alpha)*(diff.seconds-run_avg)
#         start_time = datetime.datetime.now()

        
        

        

    print(f"Current File: {fileName} \t Index: {ii}/{num_files-1} \t Percentage: {100*ii/num_files:.2f}% \t Minutes to Completion: {(num_files-ii)/60:.1f}")
    

    trial = DataFile(dataPath,fileName,dataType='SIM')
    
    ## TRIAL CONDITIONS
    vel_IC,phi_IC = trial.grab_vel_IC_2D_angle()
    trial_num = trial.trialNum

    ## RL PARAMETERS
    alpha_mu,alpha_sigma,mu_0,sigma_0 = trial.grab_RLParams()
    policy,sigma = trial.grab_finalPolicy()
    RREV_threshold,My_d = policy
    RREV_sig,My_d_sig = sigma

    ## LANDING RATES
    landing_rate_4_leg,landing_rate_2_leg,_ = trial.landing_rate()


    ## FLIP DATA
    RREV_trigger_mean,RREV_trigger_std,_ = trial.grab_flip_state_trial('RREV')
    OF_y_trigger_mean,OF_y_trigger_std,_ = trial.grab_flip_state_trial('OF_y')

    ## IMPACT DATA
    impact_force_x_mean,impact_force_x_std,_ = trial.grab_impact_force_trial('x')
    impact_force_z_mean,impact_force_z_std,_ = trial.grab_impact_force_trial('z')
    impact_eul_mean,impact_eul_std,_ = trial.grab_impact_eul_trial(eul_type='eul_y')
    t_delta_mean,t_delta_std,_ = trial.trigger2impact_trial()


    df_list.append((
        vel_IC, phi_IC, trial_num,
        alpha_mu,alpha_sigma,
        mu_0,sigma_0,
        RREV_threshold,RREV_sig,
        My_d,My_d_sig,
        landing_rate_4_leg,landing_rate_2_leg,
        RREV_trigger_mean,RREV_trigger_std,
        OF_y_trigger_mean,OF_y_trigger_std,

        impact_force_x_mean,impact_force_x_std,
        impact_force_z_mean,impact_force_z_std,
        impact_eul_mean,impact_eul_std,
        t_delta_mean,t_delta_std,
    ))

#         df_list.append((
#             vz_d,vx_d,trial_num,landing_rate,
#             RREV_threshold,G1,
#             RREV_sig,G1_sig,
#             RREV_trigger,OF_y,
#             My_d,impact_eul,impact_tdelta,
#             alpha_mu,alpha_sigma,
#             mu_ini,sigma_ini,
#             ))

#         end_time = datetime.datetime.now()

#     except:
# #         send2trash.send2trash(dataPath+fileName)
#         pass

    


print()

master_df = pd.DataFrame(df_list,columns=(
    'vel_IC','phi_IC','trial_num',
    'alpha_mu','alpha_sigma',
    'mu_0','sigma_0',
    'RREV_threshold','RREV_sig',
    'My_d','My_d_sig',
    'landing_rate_4_leg','landing_rate_2_leg',
    'RREV_trigger_mean','RREV_trigger_std',
    'OF_y_trigger_mean','OF_y_trigger_std',

    'impact_force_x_mean','impact_force_x_std',
    'impact_force_z_mean','impact_force_z_std',
    'impact_eul_mean','impact_eul_std',
    't_delta_mean','t_delta_std',
))
print(master_df)
master_df.sort_values(['vel_IC','phi_IC','trial_num'],ascending=[1,1,1],inplace=True)
# master_df.to_csv('XNL_2-Policy_Summary.csv',index=False)