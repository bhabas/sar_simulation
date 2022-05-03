
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from Data_Analysis import DataFile
# os.system("clear")

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_logging/local_logs/"

Vel = 2.5
phi = 90
trial = 2


# fileName = "EM_PEPG--Vd_3.50--phi_60.00--trial_03.csv"
# fileName = f"EM_PEPG--Vd_{Vel:.2f}--phi_{phi:.2f}--trial_{int(trial):02d}.csv"
# fileName = "My_6.00_Calibration_Test-3.csv"
fileName = "EM_PEPG--Vd_3.50--phi_90.00--trial_00--NL.csv"
trial = DataFile(dataPath,fileName,dataType='Sim')

k_ep = 0
k_run = 0

trial.plot_convg(saveFig=True)

