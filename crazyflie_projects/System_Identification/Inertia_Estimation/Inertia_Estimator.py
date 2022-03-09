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


## OPEN DATAFILE
dataPath = f"{BASE_PATH}/crazyflie_projects/System_Identification/Inertia_Estimation/Logs/"
fileName = "BaseModel_Iyy_Log-4.csv"
trial = DataFile(dataPath,fileName,dataType='EXP')

def reject_outliers(data, m = 2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.0
    return data[s<m]

## GRAB ANGULAR VELOCITY
t = trial.grab_stateData(k_ep=0,k_run=0,stateName=['t'])
x = trial.grab_stateData(k_ep=0,k_run=0,stateName=['wz'])


## COLLECT AVERAGE PERIOD FROM TIME BETWEEN PEAKS
peak_idx,_ = find_peaks(x.flatten(), distance=35)

plt.plot(t,x)
plt.plot(t.flatten()[peak_idx], x.flatten()[peak_idx],'x')
plt.plot(t,np.zeros_like(t), "--", color="gray")
plt.show()


T_list = reject_outliers(np.diff(t.flatten()[peak_idx]))
print(T_list)
T_avg = np.mean(T_list)


m = 0.0343  # [kg]
l = 0.572   # [m]
D = 0.031   # [m]
g = 9.8066  # [m/s^2]

I_est = m*g*pow(D*T_avg,2)/(pow(4*np.pi,2)*l)

print(f"T_avg: {T_avg:0.4f}")
print(f"I_est: {I_est:0.4E}")




