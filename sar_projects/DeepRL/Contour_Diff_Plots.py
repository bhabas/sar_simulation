## STANDARD IMPORTS
import os
from datetime import datetime,timedelta
import numpy as np
import pandas as pd
import torch as th
import yaml
import pandas as pd
import csv
import time 
import rospy
import rospkg
import glob
import boto3

from collections import deque

import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.interpolate import griddata


LOG_DIR = "/home/bhabas/GoogleDrive/Grad_Research/Papers/ICRA_25/Figures/2D_3D_Model_Comparison/A1_0deg"
LOG_1 = "2D"
LOG_2 = "3D"



fileName = "PolicyPerformance_Data.csv"
filePath_1 = os.path.join(LOG_DIR,LOG_1,fileName)
filePath_2 = os.path.join(LOG_DIR,LOG_2,fileName)

PlaneAngle = 0
V_B_O_cutoff = -PlaneAngle + 15


## READ CSV FILE
df_1 = pd.read_csv(filePath_1,sep=',',comment="#")
df_2 = pd.read_csv(filePath_2,sep=',',comment="#")

cols_to_keep = [col for col in df_1.columns if not col.startswith('--')]
df_1 = df_1[cols_to_keep]
df_2 = df_2[cols_to_keep]


weight_vector = np.array([1.0, 0.75, 0.5, 0.25, 0.0, 0.0])
df_1['R_legs'] = df_1.iloc[:, -6:].dot(weight_vector)
df_2['R_legs'] = df_2.iloc[:, -6:].dot(weight_vector)


df_1.drop(["reward_vals","NN_Output_trg","a_Rot_scale"],axis=1,inplace=True)
df_2.drop(["reward_vals","NN_Output_trg","a_Rot_scale"],axis=1,inplace=True)



df_1 = df_1.groupby(["V_mag","V_angle","Plane_Angle"]).mean().round(3).reset_index()
df_1.query(f"Plane_Angle == {PlaneAngle}",inplace=True)
df_1.query(f"V_mag >= 1.0",inplace=True)
# df_1.query(f"V_angle >= {V_B_O_cutoff}",inplace=True)


df_2 = df_2.groupby(["V_mag","V_angle","Plane_Angle"]).mean().round(3).reset_index()
df_2.query(f"Plane_Angle == {PlaneAngle}",inplace=True)
df_2.query(f"V_mag >= 1.0",inplace=True)
# df_2.query(f"V_angle >= {V_B_O_cutoff}",inplace=True)
    
    
## COLLECT DATA
R_1 = df_1.iloc[:]['V_mag']
Theta_1 = df_1.iloc[:]['V_angle']-PlaneAngle
C_1 = df_1.iloc[:]['R_legs']

R_2 = df_2.iloc[:]['V_mag']
Theta_2 = df_2.iloc[:]['V_angle']-PlaneAngle
C_2 = df_2.iloc[:]['R_legs']



## DEFINE INTERPOLATION GRID
R_list_1 = np.linspace(R_1.min(),R_1.max(),num=50,endpoint=True).reshape(1,-1)
Theta_list_1 = np.linspace(Theta_1.min(),Theta_1.max(),num=50,endpoint=True).reshape(1,-1)
R_grid_1, Theta_grid_1 = np.meshgrid(R_list_1, Theta_list_1)

## INTERPOLATE DATA
LR_interp_1 = griddata((R_1, Theta_1), C_1, (R_list_1, Theta_list_1.T), method='linear')
LR_interp_1 += 0.001


## DEFINE INTERPOLATION GRID
R_list_2 = np.linspace(R_2.min(),R_2.max(),num=50,endpoint=True).reshape(1,-1)
Theta_list_2 = np.linspace(Theta_2.min(),Theta_2.max(),num=50,endpoint=True).reshape(1,-1)
R_grid_2, Theta_grid_2 = np.meshgrid(R_list_2, Theta_list_2)

## INTERPOLATE DATA
LR_interp_2 = griddata((R_2, Theta_2), C_2, (R_list_2, Theta_list_2.T), method='linear')
LR_interp_2 += 0.001

## INIT PLOT INFO
fig = plt.figure(figsize=(4,4))
ax = fig.add_subplot(projection='polar')
cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=0,vmax=1)

ax.contourf(np.radians(Theta_grid_1),R_grid_1,np.abs(LR_interp_1-LR_interp_2),cmap=cmap,norm=norm,levels=60)

ax.set_xticks(np.radians(np.arange(0-PlaneAngle,180-PlaneAngle+15,15)))
ax.set_thetamin(max(0-PlaneAngle,-90))
ax.set_thetamax(min(180-PlaneAngle,90))

ax.set_rticks([0.0,1.0,2.0,3.0,4.0,5.0])
ax.set_rmin(0)
ax.set_rmax(R_1.max())
ax.set_xticklabels([])  # Remove x-axis labels
ax.set_yticklabels([])  # Remove y-axis labels

# plotName = f"Landing_Rate_diffFig_PlaneAngle_{PlaneAngle:.0f}_NoText.pdf"
# plotPath = os.path.join(LOG_DIR,plotName)
# plt.savefig(plotPath,dpi=300)

plotName = f"Landing_Rate_diffFig_PlaneAngle_{PlaneAngle:.0f}_NoText.png"
plotPath = os.path.join(LOG_DIR,plotName)
plt.savefig(plotPath,dpi=300)


# plt.show(block=True)
