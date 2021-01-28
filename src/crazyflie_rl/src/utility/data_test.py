
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")

vz = 4.00
vx = 2.75
trialNum = 0
agent = "EM_PEPG"

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/local_files/data/Wide-Short_Data_1-27-21/"
fileName = f"{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trialNum}.csv"









trial = DataFile(dataPath,fileName)

k_ep = 16
k_run = 5
# trial.plot_eulerData(k_ep,k_run,'eul_y')
trial.plotSummary()
# trial.plot_rewardData()
