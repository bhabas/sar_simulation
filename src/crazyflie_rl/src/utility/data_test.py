
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")

vz = 3.5
vx = 1.5
trialNum = 0

filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/local_files/data/Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trialNum}.csv"
# filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/log/Vz_{vz}--Vx_{vx}--trial_{trialNum}.csv"
filepath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/EM_PEPG--Vz_4.00--Vx_1.00--trial_1.csv"



trial = DataFile(filepath)

k_ep = 19
k_run = 9

# print(trial.grab_eulerData(k_ep,k_run)[0])
print(trial.grab_impact_state(k_ep,k_run,'vx'))
trial.plot_state(k_ep,k_run,'x')
