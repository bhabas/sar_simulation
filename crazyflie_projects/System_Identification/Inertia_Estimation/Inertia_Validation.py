'''
Estimate inertia about each axis via bifilar pendulum method.
Time between peaks is measured via the angular velocity and used in the equation.
Method is highlighted in: "The Experimental Determination of the Moment of Inertia of a Model Airplane - Michael Koken"

'''

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks


import sys
import os
import rospkg

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(0,BASE_PATH)
from crazyflie_logging.data_analysis.Data_Analysis import DataFile

def reject_outliers(data, m = 2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.0
    return data[s<m]

## OPEN DATAFILE
dataPath = f"{BASE_PATH}/crazyflie_projects/System_Identification/Inertia_Estimation/Logs/Exp_Logs/"
fileName = "BaseModel_Ixx_Log-2_Sim.csv"
trial = DataFile(dataPath,fileName,dataType='EXP')


## GRAB ANGULAR VELOCITY
t = trial.grab_stateData(k_ep=0,k_run=0,stateName=['t'])
wx = trial.grab_stateData(k_ep=0,k_run=0,stateName=['wx'])
z = trial.grab_stateData(k_ep=0,k_run=0,stateName=['z'])



## COLLECT AVERAGE PERIOD FROM TIME BETWEEN PEAKS

plt.plot(t,wx)
plt.plot(t,z*20)
plt.plot(t,np.zeros_like(t), "--", color="gray")
plt.show()

# plt.plot(t,z*200)
# plt.plot(t[1:],np.diff(wy.flatten())/np.diff(t.flatten()))
# plt.show()




