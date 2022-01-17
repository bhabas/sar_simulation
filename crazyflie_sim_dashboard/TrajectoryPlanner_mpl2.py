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
df = df.query('landing_rate_4_leg >= 0.7')
# arr= df.iloc[:][["OF_y_flip_mean","RREV_threshold","flip_d_mean"]]

## SYSTEM CONSTANTS
h_ceil = 2.1
z_0 = 0.4
RREV_max = 8.0

## INITAL VALUES
vel = 2.0
theta = 90
vel_x = vel*np.cos(np.radians(theta))
vel_z = vel*np.sin(np.radians(theta))


## TIME VALUES
t_c = (h_ceil - z_0)/vel_z - 1/RREV_max # t_cutoff
t = np.linspace(0,t_c,50)

## SENSORY VALUES
d = h_ceil - (z_0 + vel_z*t)
vel_z = vel_z*np.ones_like(t)
vel_x = vel_x*np.ones_like(t)


## DEFINE FIGURE
fig = plt.figure()

cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=0,vmax=1.0)

ax = fig.add_subplot(111, projection='3d')
ax.scatter(df["vx"],df["vz"],df["flip_d_mean"],c=df["landing_rate_4_leg"],cmap=cmap,norm=norm)
graph, = ax.plot(vel_x,vel_z,d)

ax.set_xlim(0,4)
ax.set_ylim(0,4)
ax.set_zlim(0,2)

ax.set_xlabel("Vel_x")
ax.set_ylabel("Vel_z")
ax.set_zlabel("d_ceil")

plt.subplots_adjust(left=0.25,bottom=0.25) # Shift figure for room for plots



## DEFINE SLIDERS
ax_theta = plt.axes([0.25, 0.1, 0.65, 0.03])
theta_slider = Slider(
    ax=ax_theta,
    label='Theta [deg]',
    valmin=20,
    valmax=90,
    valinit=theta,
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

    vel = vel_slider.val
    theta = theta_slider.val

    vel_x = vel*np.cos(np.radians(theta))
    vel_z = vel*np.sin(np.radians(theta))

    t_c = (h_ceil - z_0)/vel_z - 1/RREV_max
    t = np.linspace(0,t_c,50)


    d = h_ceil - (z_0 + vel_z*t)
    vel_z = vel_z*np.ones_like(t)
    vel_x = vel_x*np.ones_like(t)


    graph.set_data(vel_x,vel_z)
    graph.set_3d_properties(d)


theta_slider.on_changed(update)
vel_slider.on_changed(update)



plt.show()