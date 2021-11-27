
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_data'))
sys.path.insert(0,BASE_PATH)
# print(sys.path)


from crazyflie_data.data_analysis.Data_Analysis import DataFile
# os.system("clear")

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/local_logs/"

Vel = 3.0
phi = 80
trial = 2


# fileName = "EM_PEPG--Vd_3.50--phi_60.00--trial_03.csv"
# fileName = f"EM_PEPG--Vd_{Vel:.2f}--phi_{phi:.2f}--trial_{int(trial):02d}.csv"
# fileName = "My_6.00_Calibration_Test-3.csv"
fileName = "Policy_Mapping--vel_3.00--phi_90.00--trial_01--WL.csv"
trial = DataFile(dataPath,fileName,dataType='SIM')

k_ep = 0
k_run = 0

# trial.plot_convg(saveFig=True)

