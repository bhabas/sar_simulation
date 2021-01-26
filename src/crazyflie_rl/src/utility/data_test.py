
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")

vz = 4.00
vx = 0.00
trialNum = 2
agent = "EM_PEPG"

# filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/local_files/data/Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trialNum}.csv"
# filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/log/Vz_{vz}--Vx_{vx}--trial_{trialNum}.csv"
# filepath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/EM_PEPG--Vz_4.00--Vx_1.00--trial_1.csv"
filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trialNum}.csv"



trial = DataFile(filepath)

k_ep = 16
k_run = 5


# trial.plot_rewardData()
# trial.plot_state(k_ep,k_run,'z')

print(trial.landing_rate())
trial.landing_plot()