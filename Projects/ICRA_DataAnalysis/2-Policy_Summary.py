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
    
    vel_IC,phi_IC = trial.grab_vel_IC_2D_angle()
    trial_num = trial.trialNum

    landing_rate_4_leg,landing_rate_2_leg,_ = trial.landing_rate()

    alpha_mu,alpha_sigma,mu_0,sigma_0 = trial.grab_RLParams()

#         policy,sigma = trial.grab_finalPolicy()
#         RREV_threshold,G1 = policy
#         RREV_sig,G1_sig = sigma

#         RREV_trigger = trial.grab_RREV_tr_trial()
#         OF_y = trial.grab_OF_y_trial()

#         My_d = trial.grab_My_d_trial()
#         impact_eul = trial.grab_impact_eul_trial('eul_y')
#         impact_tdelta = trial.trigger2impact_trial()


    df_list.append((vel_IC, phi_IC, trial_num,
                    landing_rate_4_leg,landing_rate_2_leg,
                    alpha_mu,alpha_sigma,
                    mu_0,sigma_0))

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
# master_df = pd.DataFrame(df_list,columns=(
#     'vz_d','vx_d','trial_num','landing_rate',
#     'RREV_threshold','G1',
#     'RREV_sig','G1_sig',
#     'RREV_trigger','OF_y',
#     'My_d','impact_eul','impact_tdelta',
#     'alpha_mu','alpha_sigma',
#     'mu_ini','sigma_ini',
# ))

master_df = pd.DataFrame(df_list,columns=(
    'V_d','phi_d','trial_num',
    'landing_rate_4_leg','landing_rate_2_leg',
    'alpha_mu','alpha_simga',
    'mu_0','sigma_0',
))
print(master_df)
master_df.sort_values(['V_d','phi_d','trial_num'],ascending=[1,1,1],inplace=True)
# master_df.to_csv('XNL_2-Policy_Summary.csv',index=False)