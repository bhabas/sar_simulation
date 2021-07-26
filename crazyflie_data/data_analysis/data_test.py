
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from Data_Analysis import DataFile
os.system("clear")

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/logs/"

fileName = "EM_PEPG--Vd_2.65--phi_90.00--trial_02--SIM_FindIssue.csv"

trial = DataFile(dataPath,fileName)

k_ep = 0
k_run = 0

# trial.plot_rewardData()
# trial.plot_state_spread_flip('RREV',N=1)
# trial.grab_policy(k_ep,k_run)
trial.plot_state_correlation(['RREV','z'])
# trial.grab_impact_time(k_ep,k_run)


