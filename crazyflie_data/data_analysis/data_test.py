
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from Data_Analysis import DataFile
os.system("clear")

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/logs/EXP_Logs/"

# fileName = "EM_PEPG--Vd_2.50--phi_90.00--trial_00_Partial_Success--EXP.csv"
fileName = "My_6.00_Calibration_Test-3.csv"

trial = DataFile(dataPath,fileName,dataType='EXP')

k_ep = 0
k_run = 0

vz = trial.grab_stateData(k_ep,k_run,['vz'])
f = trial.grab_stateData(k_ep,k_run,['F_thrust'])
t = np.arange(0,len(vz)*0.01,0.01)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(t,vz)
ax.plot(t,f)
ax.grid()

plt.show()

# trial.plot_rewardData()
# trial.plot_policy_convg()
# trial.plot_state(k_ep,k_run,['vz'])
# trial.plot_state_spread_flip('RREV',N=1)
# trial.grab_policy(k_ep,k_run)
# trial.plot_state_correlation(['RREV','z'])
# trial.grab_impact_time(k_ep,k_run)


