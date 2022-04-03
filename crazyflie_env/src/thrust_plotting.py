import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


df = pd.read_csv('thrust.csv')
df = df.query('t > 14')

## SYSTEM CONSTANTS
k_f = 2.2e-8    # Thrust Coefficient [N/(rad/s)^2]
g2Newton = 1000/9.81
df2 = pd.read_csv('/home/bhabas/catkin_ws/src/crazyflie_experiment/crazyflie_projects_exp/System_Identification/Motor_Thrusts/logs/MS_Decay_6.csv')
df2['thrust'] = (df2["omega"].to_numpy()**2)*k_f*g2Newton

fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(df['t'].to_numpy(),df['thrust_input'].to_numpy())
ax.plot(df['t'].to_numpy(),df['thrust_actual'].to_numpy())
ax.plot(df2['time'].to_numpy()+10-1.47,df2['thrust'].to_numpy()-0.32+0.1217)
ax.grid()

plt.show()