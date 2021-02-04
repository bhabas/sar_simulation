
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
# os.system("clear")

vz = 3.5
vx = 0.75
trialNum = 1
agent = "EM_PEPG"




dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/"
# fileName = f"{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{int(trialNum):02d}.csv"
fileName = "EM_PEPG--Vz_3.50--Vx_0.75--trial_1.csv"





trial = DataFile(dataPath,fileName)

k_ep = 19
k_run = 9

print(trial.trigger2impact_trial())
trial.plotSummary()

print(trial.grab_RREV_tr_trial())
