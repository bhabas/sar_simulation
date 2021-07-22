
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from Data_Analysis import DataFile
os.system("clear")

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/logs/"

fileName_SIM = "EM_PEPG--Vd_2.65--phi_90.00--trial_03--SIM.csv"
fileName_EXP = "EM_PEPG--Vd_2.50--phi_90.00--trial_00--EXP.csv"

trial_sim = DataFile(dataPath,fileName_SIM)
trial_exp = DataFile(dataPath,fileName_EXP)

k_ep = 0
k_run = 1

np.set_printoptions(suppress=True)


# trial.grab_impact_time(k_ep,k_run)


