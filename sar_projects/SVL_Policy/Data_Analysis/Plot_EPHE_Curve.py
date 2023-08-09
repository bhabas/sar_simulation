import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os

## ADD SAR_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(0,BASE_PATH)
from crazyflie_logging.data_analysis.Data_Analysis import DataFile

dataPath = f"{BASE_PATH}/crazyflie_logging/local_logs/NL_DR_Data"
fileName = f"EPHE--Vd_2.70--phi_60.00--trial_01--NL--DR.csv"
filePath = os.path.join(dataPath,fileName)

trial = DataFile(dataPath,fileName,dataType='Sim')
k_ep_arr,mu_arr,sigma_arr = trial.grab_convg_data()


## RE-ORGANIZE POLICY DISTRIBUTION DATA
Tau_list_mu = mu_arr[:-2,0]/10        # Tau distributions are normalized to similar range as My 
Tau_list_sig = 2*sigma_arr[:-2,0]/10

My_list_mu = mu_arr[:-2,1]
My_list_sig = sigma_arr[:-2,1]

k_ep_arr = k_ep_arr[:-2]



## GENERATE PLOT FIGURE
fig = plt.figure(figsize=(6,3))
ax1 = fig.add_subplot()
ax2 = ax1.twinx()


## PLOT Tau DATA
ax1.plot(k_ep_arr,Tau_list_mu,color="tab:blue",label="Tau")
ax1.fill_between(k_ep_arr,Tau_list_mu+Tau_list_sig,Tau_list_mu-Tau_list_sig,color="tab:blue",alpha=0.4)

ax1.set_ylabel("Tau_cr [s]")
ax1.set_ylim(0,0.75)
ax1.set_yticks([0,0.2,0.4,0.6,0.8])


## PLOT My DATA
ax2.plot(k_ep_arr,My_list_mu,color="tab:orange",label="My")
ax2.fill_between(k_ep_arr,My_list_mu+My_list_sig,My_list_mu-My_list_sig,color="tab:orange",alpha=0.4)

ax2.set_ylim(0,8)
ax2.set_ylabel("My (N*mm)")
ax2.set_yticks([0,2,4,6,8])
ax2.set_yticklabels(["0.0","2.0","4.0","6.0","8.0"])

## CREATE PLOT LEGEND
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines + lines2, labels + labels2, loc="lower right",ncol=2)

## GENERAL PLOT SETTINGHS
ax1.set_title("Policy Distribution vs Episode | Vel = 2.70 (m/s)")

ax1.set_xlabel("Episode")
ax1.set_xlim(0,15)
ax1.set_xticks([0,3,6,9,12,15])
ax1.grid()



fig.tight_layout()
plt.savefig(f"EPHE_Convergence_Plot.pdf",dpi=300)
plt.show()
