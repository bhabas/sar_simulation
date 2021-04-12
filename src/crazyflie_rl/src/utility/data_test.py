
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from data_analysis import DataFile
# os.system("clear")


vz = 3.00
phi = 80
trialNum = 0
agent = "EM_PEPG"


dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/"
fileName = f"{agent}--Vd_{vz:.2f}--phi_{phi:.2f}--trial_{int(trialNum):02d}.csv"







trial = DataFile(dataPath,fileName)

k_ep = 19
k_run = 6
print(
    trial.trigger2impact(k_ep,k_run)
)
# trial.plot_flip_states('vz')


