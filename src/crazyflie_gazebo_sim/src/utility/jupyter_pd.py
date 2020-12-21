import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation
import os

from data_analysis import DataFile
os.system("clear")

filepath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/local_files/data/Vz_3.5--Vx_2.5--trial_0.csv"

trial_df = DataFile(filepath)

k_ep = 13
k_run = 6


run_df = trial_df.select_run(k_ep, k_run)