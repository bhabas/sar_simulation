import os,time,datetime
import pandas as pd
import send2trash


## CREATE LINK TO DATAPATH MODULE
import sys
sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility')
from data_analysis import DataFile


dataPath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/Wide-Long_2-Policy/"
df_list = []
num_files = len(os.listdir(dataPath))

run_avg = 0
run_avg_prev = 0
start_time = datetime.datetime.now()
end_time = datetime.datetime.now()


## ITER OVER ALL FILES IN DIR
for ii,fileName in enumerate(os.listdir(dataPath)): # Iter over all files in dir

    try:
        ## PROGRESS PRINTING (BASIC STUFF STARTING FOR TIME ESTIMATION)
        

        diff = end_time - start_time
        alpha = 0.85
        run_avg = run_avg + 1/10*(diff.seconds-run_avg)
        start_time = datetime.datetime.now()

        
        

        

        print(f"Current File: {fileName} \t Index: {ii}/{num_files} \t Percentage: {100*ii/num_files:.2f}% \t Minutes to Completion: {run_avg*(num_files-ii)/60:.1f}")
        

        trial = DataFile(dataPath,fileName)
        
        vx_d,_,vz_d = trial.v_d
        trial_num = fileName[-5]

        landing_rate = trial.landing_rate()

        alpha_mu,alpha_sigma,mu_ini,sigma_ini = trial.grab_RLPararms()

        policy,sigma = trial.grab_finalPolicy()
        RREV_threshold,G1 = policy
        RREV_sig,G1_sig = sigma

        RREV_trigger = trial.grab_RREV_tr_trial()
        OF_y = trial.grab_OF_y_trial()

        My_d = trial.grab_My_d_trial()
        impact_eul = trial.grab_impact_eul_trial('eul_y')
        impact_tdelta = trial.trigger2impact_trial()
    



        df_list.append((
            vz_d,vx_d,trial_num,landing_rate,
            RREV_threshold,G1,
            RREV_sig,G1_sig,
            RREV_trigger,OF_y,
            My_d,impact_eul,impact_tdelta,
            alpha_mu,alpha_sigma,
            mu_ini,sigma_ini,
            ))

        end_time = datetime.datetime.now()

    except:
        send2trash.send2trash(dataPath+fileName)
        pass

    


            
master_df = pd.DataFrame(df_list,columns=(
    'vz_d','vx_d','trial_num','landing_rate',
    'RREV_threshold','G1',
    'RREV_sig','G1_sig',
    'RREV_trigger','OF_y',
    'My_d','impact_eul','impact_tdelta',
    'alpha_mu','alpha_sigma',
    'mu_ini','sigma_ini',
))
print(master_df)
master_df.sort_values(['vz_d','vx_d','trial_num'],ascending=[1,1,1],inplace=True)
master_df.to_csv('WL_2-Policy_Summary.csv',index=False)