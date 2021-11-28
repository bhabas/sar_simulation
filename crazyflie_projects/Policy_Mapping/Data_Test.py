
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_data'))
sys.path.insert(0,BASE_PATH)


from crazyflie_data.data_analysis.Data_Analysis import DataFile
# os.system("clear")

dataPath = f"crazyflie_projects/Policy_Mapping/Data/Policy_Mapping--Sample/"

Vel = 3.0
phi = 80
trial = 2



fileName = "Policy_Mapping--vel_3.00--phi_60.00--trial_04--WL.csv"
trial = DataFile(dataPath,fileName,dataType='SIM')

k_ep = 0
k_run = 0

print(trial.grab_impact_eul(k_ep,k_run))

# trial.plot_convg(saveFig=True)

