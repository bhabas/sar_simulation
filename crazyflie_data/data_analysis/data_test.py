
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from Data_Analysis import DataFile
os.system("clear")

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/logs/"

fileName = "EM_PEPG--Vd_2.65--phi_90.00--trial_00--SIM.csv"

trial = DataFile(dataPath,fileName)

k_ep = 19
k_run = 5

# trial.plot_rewardData()
trial.plot_state_spread_flip('RREV')
# trial.grab_impact_time(k_ep,k_run)


