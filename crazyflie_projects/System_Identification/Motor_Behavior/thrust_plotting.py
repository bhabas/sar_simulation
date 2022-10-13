import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


df_sim = pd.read_csv('thrust.csv')
# df_sim = df_sim.query('t > 5')
df_sim['t'] = df_sim['t']-5.1+0.13


## SYSTEM CONSTANTS
k_f = 2.2e-8    # Thrust Coefficient [N/(rad/s)^2]
g2Newton = 1000/9.81
df_exp = pd.read_csv('/home/bhabas/catkin_ws/src/crazyflie_experiment/crazyflie_projects_exp/System_Identification/Motor_Thrusts/logs/MS_Decay_10.csv')
df_exp['thrust'] = (df_exp["omega"].to_numpy()**2)*k_f*g2Newton
df_exp['time'] = df_exp['time']-10.1

fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(df_sim['t'].to_numpy(),df_sim['thrust_input'].to_numpy(),label='Thrust Input')
ax.plot(df_exp['time'].to_numpy()-0.1667,df_exp['thrust'].to_numpy(),label='Thrust Exp.')
ax.plot(df_sim['t'].to_numpy(),df_sim['thrust_actual'].to_numpy(),label='Thrust Sim.')

ax.set_xlabel("Time [s]")
ax.set_ylabel("Thrust [g]")
ax.grid()
ax.legend()

plt.show()