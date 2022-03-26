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
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects_exp'))
sys.path.insert(0,BASE_PATH)
from Data_Analysis import DataFile

def reject_outliers(data, m = 2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.0
    return data[s<m]

## OPEN DATAFILE
dataPath = f"{BASE_PATH}/crazyflie_projects_exp/System_Identification/Inertia_Estimation/Logs/Exp_Logs/"
fileName = "BaseModel_VerticalTraj_Exp_0.5az.csv"
fileName2 = "BaseModel_VerticalTraj_Sim_0.5az.csv"

# fileName = "BaseModel_VerticalTraj_Exp.csv"
# fileName2 = "BaseModel_VerticalTraj_Sim.csv"

trial = DataFile(dataPath,fileName,dataType='EXP')
trial2 =  DataFile(dataPath,fileName2,dataType='EXP')


## GRAB ANGULAR VELOCITY
t = trial.grab_stateData(k_ep=0,k_run=0,stateName=['t'])
t = t-np.min(t)
wx = trial.grab_stateData(k_ep=0,k_run=0,stateName=['wx'])
z = trial.grab_stateData(k_ep=0,k_run=0,stateName=['vz'])

t2 = trial2.grab_stateData(k_ep=0,k_run=0,stateName=['t'])
# t2 = t2-np.min(t2)-0.066#-7.828
t2 = t2-np.min(t2)+0.67
wx2 = trial2.grab_stateData(k_ep=0,k_run=0,stateName=['wx'])
z2 = trial2.grab_stateData(k_ep=0,k_run=0,stateName=['vz'])




## COLLECT AVERAGE PERIOD FROM TIME BETWEEN PEAKS

fig = plt.figure()
ax = fig.add_subplot(111)
# ax.plot(t,wx,label='Wx_exp')
# ax.plot(t2,wx2,label='Wx_sim')
# ax.set_ylabel("Wx [rad/s]")

ax.plot(t,z,label='Z_Exp')
ax.plot(t2,z2,label='Z_Sim')
ax.set_ylabel("Z -Vel [m/s]")
ax.plot(t,np.zeros_like(t), "--", color="gray")
ax.legend()

ax.grid()
plt.show()

# plt.plot(t,z*200)
# plt.plot(t[1:],np.diff(wy.flatten())/np.diff(t.flatten()))
# plt.show()




