
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
# os.system("clear")

vz = 3.5
vx = 2.25
trialNum = 3
agent = "EM_PEPG"




dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/Wide-Short_PureMoment_Data_1-29-21/"
fileName = f"{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trialNum}.csv"




trial = DataFile(dataPath,fileName)

k_ep = 19
k_run = 9

print(trial.trigger2impact_trial())
trial.plotSummary()

print(trial.grab_RREV_tr_trial())
