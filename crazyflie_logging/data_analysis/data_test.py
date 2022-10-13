
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

k_ep = 0
k_run = 0

fig = plt.figure()
ax = fig.add_subplot(111)
for ii in range(24,29):

    fileName = f"Control_Playground--trial_{ii}--NL.csv"
    trial = DataFile(dataPath,fileName,dataType='Exp')


    if ii == 24:
        x_d = trial.grab_stateData(k_ep,k_run,['x_d.x'])
        z_d = trial.grab_stateData(k_ep,k_run,['x_d.z'])
        ax.plot(x_d,z_d)

    pos_x = trial.grab_stateData(k_ep,k_run,['x'])
    pos_z = trial.grab_stateData(k_ep,k_run,['z'])


    ax.plot(pos_x,pos_z)


ax.grid()
plt.show()

# trial.plot_convg(saveFig=True)

