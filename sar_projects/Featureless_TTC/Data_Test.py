
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(0,BASE_PATH)


from crazyflie_logging.data_analysis.Data_Analysis import DataFile
# os.system("clear")

dataPath = f"{BASE_PATH}/crazyflie_logging/local_logs/"
fileName = "EM_PEPG--Vd_2.50--phi_90.00--trial_24--NL--NL.csv"
trial = DataFile(dataPath,fileName,dataType='SIM')

k_ep = 0
k_run = 0
trial.plot_policy_convg()


