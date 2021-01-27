
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")

vz = 4.00
vx = 2.75
trialNum = 2
agent = "EM_PEPG"

filepath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/{agent}--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trialNum}.csv"



trial = DataFile(filepath)

k_ep = 16
k_run = 5
# trial.plot_eulerData(k_ep,k_run,'eul_y')
trial.plot_traj2(k_ep,k_run)

