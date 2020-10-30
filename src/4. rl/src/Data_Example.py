
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from utility import data_module as DM



## EPISODE AND RUN NUMBER ##
k_ep = 0
k_run = 0

## GRAB RESPECTIVE DATAFRAMES ##
filepath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/src/4. rl/src/log/bhabas_2020-10-30_12:55:55.csv"
df = pd.read_csv(filepath) # create dataframe
run_df,IC_df = DM.run_data(df,k_ep,k_run) # select episode and run
IC_DF_full = DM.grab_all_IC(df)
policy_hist = DM.grab_IC(IC_DF_full,'policy')



vz = DM.grab_data(run_df,'vz') # Grab array from run_df
t = DM.grab_data(run_df,'t')
t = t-np.min(t)

# RREV_tr = DM.grab_IC(IC_df,'policy') # Grab IC from IC_df
# RREV_tr = RREV_tr[0,0]
# print(RREV_tr)

vz_d = DM.grab_IC(IC_df,'vz')
vz_d = vz_d*np.ones_like(t)

plt.plot(t,vz)
plt.plot(t,vz_d)
plt.show()
















