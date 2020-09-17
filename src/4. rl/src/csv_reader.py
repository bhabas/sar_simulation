import pandas as pd
import numpy as np
import pandas

import os
print(os.getcwd())

## This works for python interactive
# f_name = '2020-09-17 11:25:01'
# df = pd.read_csv('log/%s.csv' %(f_name))

## For typical running it needs full path for right now
path = '/home/bhabas/catkin_ws/src/crazyflie_simulation/src/4. rl/src/log/2020-09-17 11:33:39.csv'
df = pd.read_csv(path)




a = df['x'].values
a = np.around(a,3)


## Grab data by episode and run number
b = df.loc[ (df['k_ep'] == 0) & (df['k_run'] == 0)]