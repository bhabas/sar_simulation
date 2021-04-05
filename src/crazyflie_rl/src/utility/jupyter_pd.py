import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation
import os

from data_analysis import DataFile
os.system("clear")


vz = 2.5
phi = 60
trialNum = 1
agent = "EM_PEPG"


dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/"
fileName = f"{agent}--Vd_{vz:.2f}--phi_{phi:.2f}--trial_{int(trialNum):02d}.csv"
trial = DataFile(dataPath,fileName)

trial_df = trial.trial_df

k_ep = 19
k_run = 7




