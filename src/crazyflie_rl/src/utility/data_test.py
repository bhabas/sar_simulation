
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
# os.system("clear")

vz = 4.0
vx = 0.0
trialNum = 0
agent = "EM_PEPG"



dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/Narrow-Long_2-Policy/"
fileName = f"{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{int(trialNum):02d}.csv"






trial = DataFile(dataPath,fileName)

k_ep = 24
k_run = 7




trial.grab_eulerData(k_ep, k_run)

print(trial.grab_impact_eul(k_ep,k_run))
trial.plot_traj2(k_ep,k_run)

print(trial.grab_RREV_tr_trial())
