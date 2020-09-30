import pandas as pd
import numpy as np
import pandas

import os
print(os.getcwd())

## This works for python interactive
# f_name = '2020-09-17 11:25:01'
# df = pd.read_csv('log/%s.csv' %(f_name))

## For typical running it needs full path for right now
f_name = 'bhabas_2020-09-17_13:32:22'
path = '/home/bhabas/catkin_ws/src/crazyflie_simulation/src/4. rl/src/log/%s.csv' %(f_name)
df = pd.read_csv(path)




a = df['x'].values


## Grab data by episode and run number
run_data = df.loc[ (df['k_ep'] == 0) & (df['k_run'] == 1)]
x =run_data['x']