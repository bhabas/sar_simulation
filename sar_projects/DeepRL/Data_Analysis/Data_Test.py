
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os


## ADD SAR_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(0,BASE_PATH)


from crazyflie_logging.data_analysis.Data_Analysis import DataFile
# os.system("clear")

dataPath = f"{BASE_PATH}/crazyflie_logging/local_logs/"
fileName = "EPHE--Vd_3.00--phi_65.00--trial_00--NL--DR.csv"
trial = DataFile(dataPath,fileName,dataType='SIM')

k_ep = 0
k_run = 0

trial.plot_state(k_ep,k_run,['z'])
# trial.plot_convg(saveFig=True)

