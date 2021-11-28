import os,time,datetime
import pandas as pd
import numpy as np
import csv,warnings
import rospkg


## CREATE LINK TO DATAPATH MODULE
import sys
# os.system("clear")



## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_data'))
sys.path.insert(0,BASE_PATH)


from crazyflie_data.data_analysis.Data_Analysis import DataFile


dataPath = f"crazyflie_projects/Policy_Mapping/Data/Policy_Mapping--Sample/"
df_list = []
num_files = len(os.listdir(dataPath))

run_avg = 20
start_time = time.time()
end_time = time.time()
alpha = 0.15

## ITER OVER ALL FILES IN DIR
for ii,fileName in enumerate(os.listdir(dataPath)): # Iter over all files in dir

    # try:
    ## PROGRESS PRINTING (BASIC STUFF STARTING FOR TIME ESTIMATION)
    diff = end_time - start_time

    run_avg = alpha*diff + (1-alpha)*(run_avg)
    start_time = time.time()
    print(f"Current File: {fileName} \t Index: {ii}/{num_files-1} \t Percentage: {100*ii/num_files:.2f}% \t Minutes to Completion: {run_avg/60*(num_files-ii):.1f}")

    # with warnings.catch_warnings():
    #     warnings.simplefilter("ignore", category=RuntimeWarning)

    trial = DataFile(dataPath,fileName,dataType='SIM')
    
    ## TRIAL CONDITIONS
    vel_IC,phi_IC = trial.grab_vel_IC_2D_angle()
    vx,_,vz = trial.grab_vel_IC()
    trial_num = trial.trialNum

    for k_run in range(trial.k_runMax+1):
        _,_,d_ceiling,My = trial.grab_policy(k_ep=0,k_run=k_run)
        RREV_trig = vz/d_ceiling
        OFy_trig = vx/d_ceiling


        impact_eul = trial.grab_impact_eul(k_ep=0,k_run=k_run)[0][1]
        leg_contacts,impact_leg,contact_list,body_impact = trial.landing_conditions(k_ep=0,k_run=k_run)

        df_list.append((
        vel_IC, phi_IC, d_ceiling,My,
        k_run,
        vx,vz,
        RREV_trig,OFy_trig,
        impact_eul,
        leg_contacts,impact_leg,contact_list,body_impact
        ))
    


    

    end_time = time.time()
    # if ii == 1:
    #     break

    # except:
    #     # send2trash.send2trash(dataPath+fileName)
    #     end_time = time.time()
    #     print(f"Trashing file {fileName}")
    #     # pass


print()

master_df = pd.DataFrame(df_list,columns=(
    'vel_IC','phi_IC','d_ceiling','My',
    'k_run',
    'vx','vz',
    'RREV_trig','OFy_trig',
    'impact_eul',
    'leg_contacts','impact_leg','contact_list','body_impact'
))
print(master_df)
master_df = master_df.round(4)
master_df.sort_values(['vel_IC','phi_IC'],ascending=[1,1],inplace=True)

master_df.to_csv(f'Policy_Mapping_Compiled.csv',index=False,quoting=csv.QUOTE_MINIMAL)
        

