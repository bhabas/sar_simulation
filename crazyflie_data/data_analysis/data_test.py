
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

from Data_Analysis import DataFile
os.system("clear")

dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/logs/"

fileName_SIM = "EM_PEPG--Vd_2.65--phi_90.00--trial_03--SIM.csv"
fileName_EXP = "EM_PEPG--Vd_2.50--phi_90.00--trial_00--EXP.csv"

trial_sim = DataFile(dataPath,fileName_SIM)
trial_exp = DataFile(dataPath,fileName_EXP)

k_ep_exp = 0
k_run_exp = 0

k_ep_sim = 0
k_run_sim = 3

# np.set_printoptions(suppress=True)

trial_exp.grab_flip_time(k_ep_exp,k_run_exp)
trial_sim.grab_flip_time(k_ep_sim,k_run_sim)

# trial_exp.plot_state(k_ep_exp,k_run_exp,['z'])

t_exp = trial_exp.grab_stateData(k_ep_exp,k_run_exp,['t'])
z_exp = trial_exp.grab_stateData(k_ep_exp,k_run_exp,['z'])


t_sim = trial_sim.grab_stateData(k_ep_sim,k_run_sim,['t'])
z_sim = trial_sim.grab_stateData(k_ep_sim,k_run_sim,['z'])

fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(t_exp-t_exp[0],z_exp,label='Exp')
ax.plot(t_sim-t_sim[0],z_sim,label='Sim')


ax.set_xlabel('Time [s]')
ax.set_ylabel('Z [m]')
ax.grid()
ax.legend()
plt.show()



# trial.grab_impact_time(k_ep,k_run)


