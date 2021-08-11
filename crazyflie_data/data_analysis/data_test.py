
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from Data_Analysis import DataFile
# os.system("clear")

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/logs/EXP_Logs/"
dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/local_logs/ExtraNarrow-Long_2-Policy/"

Vel = 1.0
phi = 17.5
trial = 0


# fileName = "EM_PEPG--Vd_3.50--phi_60.00--trial_03.csv"
fileName = f"EM_PEPG--Vd_{Vel:.2f}--phi_{phi:.2f}--trial_{int(trial):02d}.csv"
# fileName = "My_6.00_Calibration_Test-3.csv"

trial = DataFile(dataPath,fileName,dataType='SIM')

k_ep = 15
k_run = 12

# print(trial.grab_impact_eul(k_ep, k_run))
_,_,arr = trial.grab_impact_eul_trial(landing_cutoff=0)
_,_,_,contact_list = trial.landing_rate()
# print(trial.grab_trial_data(trial.trigger2impact))


print()
# wy = trial.grab_stateData(k_ep,k_run,['wy'])
# f_pitch = trial.grab_stateData(k_ep,k_run,['My'])
# t = np.arange(0,len(wy)*0.01,0.01)
# eul = trial.grab_eulerData(k_ep, k_run, degrees=bool)[:,1]


# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.plot(t,wy)
# ax.plot(t,f_pitch)
# ax.plot(t,eul)
# ax.grid()

# plt.show()

# trial.plot_rewardData()
# trial.plot_policy_convg()

# print(trial.grab_impact_eul(k_ep,k_run))
# print(trial.grab_flip_state_trial('t'))
# trial.plot_state_correlation(stateList=['RREV','vz'],typeList=['flip','impact'])

# trial.plot_state(k_ep,k_run,['vz'])
# trial.plot_state_spread_flip('RREV',N=1)
# trial.grab_policy(k_ep,k_run)
# trial.plot_state_correlation(['RREV','z'])
# trial.grab_impact_time(k_ep,k_run)


