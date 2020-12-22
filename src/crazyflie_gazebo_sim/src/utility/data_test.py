
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")

vz = 3.5
vx = 0.0
trialNum = 0

filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/local_files/data/Vz_{vz}--Vx_{vx}--trial_{trialNum}.csv"
# filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/log/Vz_{vz}--Vx_{vx}--trial_{trialNum}.csv"



trial = DataFile(filepath)
print(trial.grab_omega_d_trial())
# trial.plot_rewardFunc()


# mu,sig = trial.grab_finalPolicy()
# print(mu)
# print(sig)

k_ep = 13
k_run = 6

# trial.plot_policy(trialNum)

# wy = trial.grab_stateData(k_ep,k_run,'wy')
# print(np.max(abs(wy)))
# trial.plot_state(k_ep,k_run,'wy')

# policy = trial.grab_policy(k_ep,k_run)

# trial.grab_omega_d(k_ep,k_run)

# print(trial.grab_finalPolicy())
# print(trial.landing_rate())
# print(trial.grab_vel_flip_trial())


# v = trial.grab_V_flip(k_ep,k_run)
print(trial.grab_impact_omega_trial())

# print(v)