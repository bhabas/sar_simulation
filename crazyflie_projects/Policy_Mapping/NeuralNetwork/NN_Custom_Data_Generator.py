import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import sys
import os
import rospkg


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(0,BASE_PATH)

DATA_PATH = f"{BASE_PATH}/crazyflie_projects/ICRA_DataAnalysis/Wide-Long_2-Policy/Wide-Long_2-Policy_Summary.csv"
df = pd.read_csv(DATA_PATH)
# df = df.query('landing_rate_4_leg >= 0.7 and 6.0 < My_d < 7.0')
# df = df.query('landing_rate_4_leg >= 0.7')

## SYSTEM CONSTANTS
H_CEIL = 2.1
Z_0 = 0.4
RREV_MAX = 8.0 # Cutoff value for plots (Practically contact at this point (0.125s))

## GENERATE MAX RREV WALL
OFy_wall = np.linspace(-20,0,11)
d_ceil_wall = np.linspace(0,1.5,11)
XX,YY = np.meshgrid(OFy_wall,d_ceil_wall)
RREV_wall = 6.5*np.ones_like(XX)
LR_wall = np.zeros_like(XX)
data_wall = np.vstack((XX.flatten(),RREV_wall.flatten(),YY.flatten(),LR_wall.flatten())).T

## GENERATE MAX VELOCITY CURVE
theta_range = np.linspace(20,90,20)
v_range = np.linspace(4.5,5.5,3)
v_range,theta_range = np.meshgrid(v_range,theta_range)
vx_range = v_range.flatten()*np.cos(np.radians(theta_range.flatten()))
vz_range = v_range.flatten()*np.sin(np.radians(theta_range.flatten()))
d_range = vz_range/5.0
RREV_range = vz_range/d_range
OFy_range = -vx_range/d_range
LR_vmax = np.zeros_like(d_range)

data_vmax = np.vstack((OFy_range.flatten(),RREV_range.flatten(),d_range.flatten(),LR_vmax.flatten())).T

data = np.vstack((data_wall,data_vmax))
df_custom = pd.DataFrame(data,columns=["OF_y_flip_mean","RREV_flip_mean","flip_d_mean","landing_rate_4_leg"])
df_custom.to_csv(f"{BASE_PATH}/crazyflie_projects/Policy_Mapping/NeuralNetwork/dataBounds.csv",index=False)
df = pd.concat([df,df_custom])

## PLOT SETUP
cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=0,vmax=1)

fig = plt.figure()
ax1 = fig.add_subplot(121,projection='3d')
ax1.scatter(df["vx"],df["vz"],df["flip_d_mean"],c=df["landing_rate_4_leg"],cmap=cmap,norm=norm)
ax1.scatter(vx_range,vz_range,d_range)

ax1.set_xlim(0,4)
ax1.set_ylim(0,4)
ax1.set_zlim(0,1.8) 

ax1.set_xlabel("Vel_x [m/s]")
ax1.set_ylabel("Vel_z [m/s]")
ax1.set_zlabel("d_ceil [m]")

ax2 = fig.add_subplot(122,projection='3d')
ax2.scatter(df["OF_y_flip_mean"],df["RREV_flip_mean"],df["flip_d_mean"],c=df["landing_rate_4_leg"],cmap=cmap,norm=norm)
# ax2.scatter(OFy_range,RREV_range,d_range)
# ax2.scatter(data_wall[:,0],data_wall[:,1],data_wall[:,2])


ax2.set_xlim(-15,0)
ax2.set_ylim(0,8)
ax2.set_zlim(0,1.8) 

ax2.set_xlabel("OFy [rad/s]")
ax2.set_ylabel("RREV [rad/s]")
ax2.set_zlabel("d_ceil [m")

plt.show()