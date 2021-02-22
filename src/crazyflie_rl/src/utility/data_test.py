
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
# os.system("clear")

vz = 3.5
vx = 1.5
trialNum = 7
agent = "EM_PEPG"




dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/ExtraNarrow-Long_2-Policy/NewData/"
fileName = f"{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{int(trialNum):02d}.csv"
# fileName = "EM_PEPG--Vz_3.50--Vx_0.75--trial_1.csv"





trial = DataFile(dataPath,fileName)

k_ep = 24
k_run = 7

print(trial.grab_OF_y(k_ep, k_run))
print(trial.grab_OF_y_trial())
# trial.plot_traj(k_ep,k_run)
trial.plotSummary()

print(trial.grab_RREV_tr_trial())
