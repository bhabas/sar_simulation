import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")

filepath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/log/Vz_3.0--Vx_1.5--trial_0.csv"

trial_df = pd.read_csv(filepath)