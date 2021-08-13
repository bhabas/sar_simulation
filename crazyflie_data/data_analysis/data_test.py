
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from Data_Analysis import DataFile
# os.system("clear")

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/logs/EXP_Logs/"
dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/local_logs/"

Vel = 2.5
phi = 90
trial = 2


# fileName = "EM_PEPG--Vd_3.50--phi_60.00--trial_03.csv"
fileName = f"EM_PEPG--Vd_{Vel:.2f}--phi_{phi:.2f}--trial_{int(trial):02d}.csv"
# fileName = "My_6.00_Calibration_Test-3.csv"

trial = DataFile(dataPath,fileName,dataType='SIM')

k_ep = 0
k_run = 0

# print(trial.grab_impact_eul(k_ep, k_run))
# print(trial.grab_trial_data(trial.trigger2impact))
t = trial.grab_stateData(k_ep,k_run,['t'])
z = trial.grab_stateData(k_ep, k_run, ['z'])
vz = trial.grab_stateData(k_ep, k_run, ['vz'])
RREV = trial.grab_stateData(k_ep, k_run, ['RREV'])

dRREV = ((2.1-z)*-9.81+vz**2)/(2.1-z)

d = -9.81/(dRREV - RREV**2)


trial.plot_state(k_ep,k_run,['vz','RREV'])
print()


fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(t,d,label='est')
ax.plot(t,2.1-z,label='actual')
# ax.plot(t,f_pitch)
# ax.plot(t,eul)
ax.grid()
ax.legend()
ax.set_xlabel('t')
ax.set_ylabel('d_ceiling')

plt.show()

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


