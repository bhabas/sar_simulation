import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation
import os

from data_analysis import DataFile
os.system("clear")


vz = 4.0
vx = 2.5
trialNum = 0
agent = "EM_PEPG"

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/Narrow-Long_2-Policy/"
fileName = f"{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{int(trialNum):02d}.csv"
trial = DataFile(dataPath,fileName)

trial_df = trial.trial_df

k_ep = 24
k_run = 6


run_df,IC_df = trial.select_run(k_ep, k_run)