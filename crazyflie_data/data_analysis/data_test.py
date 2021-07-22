
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
os.system("clear")


# vz = 3.00
# phi = 80
# trialNum = 0
# agent = "EM_PEPG"


# dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/"
# fileName = f"{agent}--Vd_{vz:.2f}--phi_{phi:.2f}--trial_{int(trialNum):02d}.csv"



dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/logs/"
fileName = "EM_PEPG--Vd_2.65--phi_90.00--trial_03--SIM_Keep.csv"
# fileName = "EM_PEPG--Vd_2.50--phi_90.00--trial_00--EXP.csv"

trial = DataFile(dataPath,fileName)

k_ep = 0
k_run = 1

np.set_printoptions(suppress=True)


trial.grab_impact_time(k_ep,k_run)


