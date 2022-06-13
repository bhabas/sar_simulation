
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from Data_Analysis import DataFile
# os.system("clear")

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_experiment/crazyflie_logging_exp/local_logs/"

Vel = 2.5
phi = 90
trial = 2


# fileName = "EM_PEPG--Vd_3.50--phi_60.00--trial_03.csv"
# fileName = f"EM_PEPG--Vd_{Vel:.2f}--phi_{phi:.2f}--trial_{int(trial):02d}.csv"
# fileName = "My_6.00_Calibration_Test-3.csv"
fileName = "Control_Playground--trial_24--NL2.csv"
trial = DataFile(dataPath,fileName,dataType='Sim')

k_ep = 0
k_run = 0

x_d = trial.grab_stateData(k_ep,k_run,['x_d.x'])
z_d = trial.grab_stateData(k_ep,k_run,['x_d.z'])

plt.figure()
plt.plot(x_d,z_d)
plt.show()

# trial.plot_convg(saveFig=True)

