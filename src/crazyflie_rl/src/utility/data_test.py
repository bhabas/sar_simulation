
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
# os.system("clear")

vz = 2.25
vx = 2.75
trialNum = 4
agent = "EM_PEPG"


dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/Wide-Short_PureMoment_Data_1-29-21/"
# dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/"
fileName = f"{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trialNum}.csv"
# fileName = "EM_PEPG--Vz_3.0--Vx_0.0--trial_1.csv"










trial = DataFile(dataPath,fileName)

k_ep = 19
k_run = 9
# print(trial.grab_impact_eul(k_ep,k_run,'eul_y'))
print(trial.grab_RREV_tr(k_ep,k_run))
# trial.plot_My_d_trial()
# trial.plot_impact_eul_trial('eul_y')
trial.plotSummary()


# trial.plot_eulerData(k_ep,k_run,'eul_y')

print(trial.grab_RREV_tr_trial())
