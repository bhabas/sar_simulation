import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import os,sys,rospkg
import numpy as np
import pandas as pd

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(0,BASE_PATH)

DATA_PATH = f"{BASE_PATH}/crazyflie_projects/ICRA_DataAnalysis/Wide-Long_2-Policy/Wide-Long_2-Policy_Summary.csv"
df = pd.read_csv(DATA_PATH)
# df = df.query('landing_rate_4_leg >= 0.7')

## SYSTEM CONSTANTS
h_ceil = 2.1
z_0 = 0.4
RREV_max = 8.0 # Cutoff value for plots (Practically contact at this point (0.125s))

## INITAL VALUES FOR PLOTTING
vel = 2.0
theta = 90
vel_x = vel*np.cos(np.radians(theta))
vel_z = vel*np.sin(np.radians(theta))
d_ceil = 0.4


## TIME VALUES
t_c = (h_ceil - z_0)/vel_z - 1/RREV_max # t_cutoff
t = np.linspace(0,t_c,50)

## VELOCITY VALUES
vel_z = vel_z*np.ones_like(t)
vel_x = vel_x*np.ones_like(t)

## SENSORY VALUES
d = h_ceil - (z_0 + vel_z*t)
RREV = vel_z/d
OFy = -vel_x/d

vel_range = np.linspace(0.5,4.0,11)
theta_range = np.linspace(90,20,10)
d_ceil_range = np.linspace(0.25,1.0,10)

XX,YY,ZZ = np.meshgrid(vel_range,theta_range,d_ceil_range)
data = np.vstack((XX.flatten(),YY.flatten(),ZZ.flatten())).T

vel_x_range = data[:,0]*np.cos(np.radians(data[:,1]))
vel_z_range = data[:,0]*np.sin(np.radians(data[:,1]))

d_range = data[:,2]
RREV_range = vel_z_range/d_range
OFy_range = -vel_x_range/d_range



## DEFINE FIGURE
fig = plt.figure(figsize=(14,7))

cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=0,vmax=1.0)

## VELOCITY-SPACE PLOT
ax = fig.add_subplot(121, projection='3d')
ax.scatter(df["vx"],df["vz"],df["flip_d_mean"],c=df["landing_rate_4_leg"],cmap=cmap,norm=norm)
POI, = ax.plot([vel_x[0]],[vel_z[0]],[d_ceil],'ko')
graph, = ax.plot(vel_x,vel_z,d)

# ax.scatter(vel_x_range,vel_z_range,d_range)


ax.set_xlim(0,4)
ax.set_ylim(0,4)
ax.set_zlim(0,2)

ax.set_xlabel("Vel_x")
ax.set_ylabel("Vel_z")
ax.set_zlabel("d_ceil")


## SENSORY-SPACE PLOT
ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(df["OF_y_flip_mean"],df["RREV_threshold"],df["flip_d_mean"],c=df["landing_rate_4_leg"],cmap=cmap,norm=norm)
POI2, = ax2.plot([OFy[0]],[RREV[0]],[d_ceil],'ko')
# ax2.scatter(OFy_range,RREV_range,d_range)
graph2, = ax2.plot(OFy,RREV,d)

ax2.set_xlim(-15,0)
ax2.set_ylim(0,8)
ax2.set_zlim(0,2)

ax2.set_xlabel("OFy")
ax2.set_ylabel("RREV")
ax2.set_zlabel("d_ceil")

plt.subplots_adjust(left=0.25,bottom=0.3) # Shift figure for room for sliders



## DEFINE SLIDERS
ax_theta = plt.axes([0.25, 0.2, 0.65, 0.03])
theta_slider = Slider(
    ax=ax_theta,
    label='Theta [deg]',
    valmin=20,
    valmax=90,
    valinit=theta,
)

ax_d = plt.axes([0.25, 0.1, 0.65, 0.03])
d_slider = Slider(
    ax=ax_d,
    label='D_ceil [deg]',
    valmin=0.0,
    valmax=1.7,
    valinit=0.5,
)

ax_vel = plt.axes([0.1, 0.25, 0.0225, 0.63])
vel_slider = Slider(
    ax=ax_vel,
    label="Velocity [m/s]",
    valmin=0,
    valmax=4,
    valinit=vel,
    orientation="vertical"
)

def update(val):

    ## UPDATE VALUES FROM SLIDERS
    vel = vel_slider.val
    theta = theta_slider.val
    d_ceil = d_slider.val

    vel_x = vel*np.cos(np.radians(theta))
    vel_z = vel*np.sin(np.radians(theta))

    ## SET POINT OF INTEREST COORDINATES FROM SLIDER
    POI.set_data([vel_x],[vel_z])
    POI.set_3d_properties([d_ceil])

    POI2.set_data([-vel_x/d_ceil],[vel_z/d_ceil])
    POI2.set_3d_properties([d_ceil])


    ## UPDATE TIME TO PLOT TRAJECTORY
    t_c = (h_ceil - z_0)/vel_z - 1/RREV_max
    t = np.linspace(0,t_c,50)

    ## UPDATE VELOCITY VALUES FOR TRAJECTORY
    vel_z = vel_z*np.ones_like(t)
    vel_x = vel_x*np.ones_like(t)
    ## UPDATE SENSORY VALUES FOR TRAJECTORY
    d = h_ceil - (z_0 + vel_z*t)
    RREV = vel_z/d
    OFy = -vel_x/d
    
    
    graph.set_data(vel_x,vel_z)
    graph.set_3d_properties(d)
    
    graph2.set_data(OFy,RREV)
    graph2.set_3d_properties(d)


theta_slider.on_changed(update)
vel_slider.on_changed(update)
d_slider.on_changed(update)

plt.show()