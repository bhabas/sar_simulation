import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

BASEPATH = "crazyflie_projects/Policy_Mapping/Data_Analysis"

df = pd.read_csv(f"{BASEPATH}/Policy_Mapping_Compiled.csv")



fig = plt.figure(1)
ax = fig.add_subplot(111,projection="3d")

X_1 = df.query('landing_success >= 0.6')['OFy_trig']
Y_1 = df.query('landing_success >= 0.6')['RREV_trig']
Z_1 = df.query('landing_success >= 0.6')['d_ceiling']
C_1 = df.query('landing_success >= 0.6')['landing_success']

X_2 = df.query('landing_success < 0.5')['OFy_trig']
Y_2 = df.query('landing_success < 0.5')['RREV_trig']
Z_2 = df.query('landing_success < 0.5')['d_ceiling']
C_2 = df.query('landing_success < 0.5')['landing_success']



cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=0,vmax=1.0)




# CREATE PLOTS AND COLOR BAR
ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.8,depthshade=False)
ax.scatter(X_2,Y_2,Z_2,c=C_2,cmap=cmap,norm=norm,alpha=0.25,depthshade=False)

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

