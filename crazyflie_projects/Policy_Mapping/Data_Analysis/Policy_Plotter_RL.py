import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

BASEPATH = "crazyflie_projects/ICRA_DataAnalysis/Wide-Long_2-Policy"

df = pd.read_csv(f"{BASEPATH}/Wide-Long_2-Policy_Summary.csv")
# df = df.query('My_d >= 3.0 and My_d < 5.0')




fig = plt.figure(1)
ax = fig.add_subplot(111,projection="3d")

X_1 = df.query('landing_rate_4_leg >= 0.6')['OF_y_flip_mean']
Y_1 = df.query('landing_rate_4_leg >= 0.6')['RREV_flip_mean']
Z_1 = 2.1 - df.query('landing_rate_4_leg >= 0.6')['flip_height_mean']
C_1 = df.query('landing_rate_4_leg >= 0.6')['My_d']

X_2 = df.query('landing_rate_4_leg < 0.5')['OF_y_flip_mean']
Y_2 = df.query('landing_rate_4_leg < 0.5')['RREV_flip_mean']
Z_2 = 2.1 - df.query('landing_rate_4_leg < 0.5')['flip_height_mean']
C_2 = df.query('landing_rate_4_leg < 0.5')['My_d']



cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=0,vmax=10.0)




# CREATE PLOTS AND COLOR BAR
ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.8,depthshade=False)
# ax.scatter(X_2,Y_2,Z_2,c=C_2,cmap=cmap,norm=norm,alpha=0.25,depthshade=False)

ax.set_xlabel('OF_y')
ax.set_xlim(-20,0)
ax.set_xticks([-20,-15,-10,-5,0])


ax.set_ylabel('RREV')
ax.set_ylim(0,8)
ax.set_yticks([0,2,4,6,8])

ax.set_zlabel('d_ceiling')
ax.set_zlim(0,1.0)
ax.set_zticks([0,0.2,0.4,0.6,0.8,1.0])

plt.show()

