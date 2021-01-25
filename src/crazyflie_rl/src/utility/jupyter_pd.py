import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation
import os

from data_analysis import DataFile
os.system("clear")

filepath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/EM_PEPG--Vz_3.5--Vx_1.5--trial_1.csv"

trial = DataFile(filepath)
trial_df = trial.trial_df

k_ep = 13
k_run = 6


run_df = trial.select_run(k_ep, k_run)