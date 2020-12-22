import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")

filepath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/local_files/data/Vz_3.5--Vx_2.5--trial_0.csv"

trial_df = pd.read_csv(filepath)