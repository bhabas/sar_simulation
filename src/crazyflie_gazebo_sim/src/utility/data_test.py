
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")

vz = 3.5
vx = 2.0
trialNum = 0

filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/log/Vz_{vz}--Vx_{vx}--trial_{trialNum}.csv"

trial_1 = DataFile(filepath)

# mu,sig = trial_1.grab_finalPolicy()
# print(mu)
# print(sig)

k_ep = 14
k_run = 9

wy = trial_1.grab_stateData(k_ep,k_run,'wy')
print(np.max(abs(wy)))
trial_1.plot_state(k_ep,k_run,'wy')

policy = trial_1.grab_policy(k_ep,k_run)

trial_1.grab_omega_d(k_ep,k_run)




v = trial_1.grab_V_flip(k_ep,k_run)
w_max = trial_1.grab_omega_flip(k_ep,k_run)

print(v)