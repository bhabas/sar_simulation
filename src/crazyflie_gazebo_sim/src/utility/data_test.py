
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")

vz = 3.0
vx = 2.5
trialNum = 2

filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/local_files/data/Vz_{vz}--Vx_{vx}--trial_{trialNum}.csv"
filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/log/Vz_{vz}--Vx_{vx}--trial_{trialNum}.csv"



trial = DataFile(filepath)

# mu,sig = trial.grab_finalPolicy()
# print(mu)
# print(sig)

k_ep = 14
k_run = 7

# wy = trial.grab_stateData(k_ep,k_run,'wy')
# print(np.max(abs(wy)))
# trial.plot_state(k_ep,k_run,'wy')

# policy = trial.grab_policy(k_ep,k_run)

# trial.grab_omega_d(k_ep,k_run)

# print(trial.grab_finalPolicy())
# print(trial.landing_rate())
print(trial.grab_vel_flip_trial())
trial.plot_rewardFunc()

# v = trial.grab_V_flip(k_ep,k_run)
# w_max = trial.grab_omega_flip(k_ep,k_run)

# print(v)