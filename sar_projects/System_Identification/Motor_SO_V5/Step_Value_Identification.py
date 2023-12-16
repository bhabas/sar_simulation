import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import os


import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_projects'))


RPM_TO_RAD_PER_SEC = 2*np.pi/60
NUM_BATT_CELLS = 6


## DATAPATHS
dataPath = f"{BASE_PATH}/sar_projects/System_Identification/Motor_SO_V5/Logs/"
fileName = f"StepsTestV2_2023-12-14_111824.csv"
filePath = os.path.join(dataPath,fileName)

## DEFINE FITTING FUNCTIONS AND SYSTEM CONSTANTS
def linear_func(x,a,b):
    return a*x + b

def quadratic_func(x,a):
    return  a*x**2

def sqrt_func(x,a,b):
    return  a*x**0.5 + b

def piecewise_func(x, a, b, c, d):
    """
    Piecewise function that is quadratic for x < x0 and linear for x >= x0.
    Parameters:
    - x0: The breakpoint between quadratic and linear
    - a, b, c: Parameters for the quadratic part (ax^2 + bx + c)
    - d: Slope for the linear part (dx)
    """
    x0 = 500
    return np.piecewise(x, [x < x0, x >= x0],
                        [lambda x: a * x**2 + b * x + c, 
                         lambda x: d * x + (a * x0**2 + b * x0 + c - d * x0)])

def piecewise_func_inv(x, a, b, c, d):
    """
    Piecewise function that is quadratic for x < x0 and linear for x >= x0.
    Parameters:
    - x0: The breakpoint between quadratic and linear
    - a, b, c: Parameters for the quadratic part (ax^2 + bx + c)
    - d: Slope for the linear part (dx)
    """
    x0 = 500
    y0 = piecewise_func(500, a, b, c, d)
    return np.piecewise(x, [x < y0, y0 <= x],
                        [lambda x: (-b + np.sqrt(b**2 - 4*a*(c-x)))/(2*a), 
                         lambda x: (x - (a * x0**2 + b * x0 + c - d * x0))/d])

def quad_func_inv(x,a,b,c):
    return  -b - np.sqrt(b**2 - 4*a*(c-x))/2*a

def linear_func_inv(x,a,b):
    return  (x-b)/a


## LOAD DATA
df = pd.read_csv(filePath,comment="#")
df = df.query("`Voltage (V)` >= 3.85*@NUM_BATT_CELLS")


## INITIALIZE PLOTS
fig = plt.figure(figsize=(14,10),constrained_layout=False)
ax_thrust = fig.add_subplot(2, 3, 1)
ax_ESC_signal = fig.add_subplot(2, 3, 2)
ax_Thrust_Cmd = fig.add_subplot(2, 3, 3)
ax_angular_speed = fig.add_subplot(2, 3, 4)
ax_torque = fig.add_subplot(2, 3, 5)
ax_current = fig.add_subplot(2, 3, 6)


## THRUST PLOT
thrust_data = df[["Thrust (gf)","Motor Optical Speed (rad/s)","Voltage (V)"]].interpolate()
ax_thrust.scatter(thrust_data["Motor Optical Speed (rad/s)"],thrust_data["Thrust (gf)"],c=thrust_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)
ax_thrust.set_title("Thrust vs Angular Speed")
ax_thrust.set_xlabel("Angular Speed [rad/s]")
ax_thrust.set_ylabel("Thrust [gf]")
ax_thrust.grid(True)

params, _ = curve_fit(
        quadratic_func,
        xdata=thrust_data["Motor Optical Speed (rad/s)"],
        ydata=thrust_data["Thrust (gf)"],
        maxfev=5000,
    )

temp_vals = np.linspace(0,thrust_data["Motor Optical Speed (rad/s)"].max(),100)
ax_thrust.plot(temp_vals,quadratic_func(temp_vals,*params),color="grey",linestyle="--",label="Quadratic Fit",zorder=0)

# ax_thrust.legend()

## ESC CMD PLOT
ESC_data = df[["Thrust (gf)","ESC signal","Voltage (V)"]]
ax_ESC_signal.scatter(ESC_data["ESC signal"],ESC_data["Thrust (gf)"],c=ESC_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)
ax_ESC_signal.set_title("ESC Cmd vs Thrust")
ax_ESC_signal.set_ylabel("Thrust (gf)")
ax_ESC_signal.set_xlabel("ESC Cmd")
ax_ESC_signal.set_xlim(-200,2200)
ax_ESC_signal.vlines(2047,0,ESC_data["Thrust (gf)"].max(),color="black",linestyles='--',label="ESC Max")
ax_ESC_signal.grid(True)


df_fit = ESC_data.query("`ESC signal` > 0")
params, _ = curve_fit(
        piecewise_func,
        xdata=df_fit["ESC signal"].to_numpy(),
        ydata=df_fit["Thrust (gf)"].to_numpy(),
        maxfev=5000,
    )

temp_vals = np.linspace(0,ESC_data["ESC signal"].max(),100)
ax_ESC_signal.plot(temp_vals,piecewise_func(temp_vals,*params),color="grey",linestyle="--",zorder=0)
# fit_label = f"a*x**(1/2) + b -> params = {np.array2string(params, precision=2)}"


## THRUST COMMAND PLOT
ESC_data = df[["Thrust (gf)","ESC signal","Voltage (V)"]]
ax_Thrust_Cmd.scatter(ESC_data["Thrust (gf)"],ESC_data["ESC signal"],c=ESC_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)
ax_Thrust_Cmd.set_title("ESC Cmd vs Thrust")
ax_Thrust_Cmd.set_xlabel("Thrust (gf)")
ax_Thrust_Cmd.set_ylabel("ESC Cmd")
ax_Thrust_Cmd.set_ylim(-200,2200)
ax_Thrust_Cmd.hlines(2047,0,ESC_data["Thrust (gf)"].max(),color="black",linestyles='--',label="ESC Max")
ax_Thrust_Cmd.grid(True)

temp_vals = np.linspace(0,ESC_data["Thrust (gf)"].max(),100)
piecewise_func_inv(500,*params)
ax_Thrust_Cmd.plot(temp_vals,piecewise_func_inv(temp_vals,*params),color="grey",linestyle="--",zorder=0)

ax_Thrust_Cmd.legend(loc="lower right")




## TORQUE PLOT  
torque_data = df[["Thrust (gf)","Torque (N·m)","Voltage (V)"]].interpolate()
ax_torque.scatter(torque_data["Torque (N·m)"],torque_data["Thrust (gf)"],c=torque_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)
ax_torque.set_title("Thrust vs Torque")
ax_torque.set_xlabel("Torque [N·m]")
ax_torque.set_ylabel("Thrust [gf]")
ax_torque.grid(True)

params, _ = curve_fit(
        linear_func,
        xdata=torque_data["Torque (N·m)"],
        ydata=torque_data["Thrust (gf)"],
        maxfev=5000,
    )

temp_vals = np.linspace(0,torque_data["Torque (N·m)"].max(),100)
ax_torque.plot(temp_vals,linear_func(temp_vals,*params),color="grey",linestyle="--",label="Linear Fit",zorder=0)
# ax_torque.legend()


## ANGULAR SPEED PLOT
angular_speed_data = df[["Torque (N·m)","Motor Optical Speed (rad/s)","Voltage (V)"]].interpolate()
ax_angular_speed.scatter(angular_speed_data["Motor Optical Speed (rad/s)"],angular_speed_data["Torque (N·m)"],c=angular_speed_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)
ax_angular_speed.set_title("Torque vs Angular Speed")
ax_angular_speed.set_xlabel("Angular Speed [rad/s]")
ax_angular_speed.set_ylabel("Torque (N·m)")
ax_angular_speed.grid(True)

params, _ = curve_fit(
        quadratic_func,
        xdata=angular_speed_data["Motor Optical Speed (rad/s)"],
        ydata=angular_speed_data["Torque (N·m)"],
        maxfev=5000,
    )

temp_vals = np.linspace(0,angular_speed_data["Motor Optical Speed (rad/s)"].max(),100)
ax_angular_speed.plot(temp_vals,quadratic_func(temp_vals,*params),color="grey",linestyle="--",label="Quadratic Fit",zorder=0)




## CURRENT PLOT
current_data = df[["Time (s)","Current (A)","Voltage (V)"]].interpolate()
ax_current.scatter(current_data["Time (s)"],current_data["Current (A)"],c=current_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)
ax_current.set_title("Current")
ax_current.set_xlabel("Time [s]")
ax_current.set_ylabel("Current [A]")
ax_current.set_ylim(0)
ax_current.grid(True)
# ax_current.legend()


fig.tight_layout()
fig.subplots_adjust(top=0.9)
fig.suptitle(f"Data from {fileName}")
fig.savefig(f"{BASE_PATH}/sar_projects/System_Identification/Motor_SO_V5/Plots/{fileName[:-4]}.png")
plt.show(block=True)

