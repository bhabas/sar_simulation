
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")

vz = 3.5
vx = 1.5
trialNum = 0

filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/local_files/data/Vz_{vz}--Vx_{vx}--trial_{trialNum}.csv"
# filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/log/Vz_{vz}--Vx_{vx}--trial_{trialNum}.csv"



trial = DataFile(filepath)

k_ep = 13
k_run = 1

print(trial.grab_impact_omega_trial())
trial.plot_eulerData(k_ep,k_run,'eul_y')

# print(v)