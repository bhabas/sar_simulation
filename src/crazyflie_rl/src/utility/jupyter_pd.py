import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation
import os

from data_analysis import DataFile
os.system("clear")


vz = 4.00
vx = 0.00
trialNum = 2
agent = "EM_PEPG"
filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trialNum}.csv"

trial = DataFile(filepath)
trial_df = trial.trial_df

k_ep = 19
k_run = 9


run_df,IC_df = trial.select_run(k_ep, k_run)