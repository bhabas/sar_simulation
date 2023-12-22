import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import os


import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_projects'))


RPM_TO_RAD_PER_SEC = 2*np.pi/60
NUM_BATT_CELLS = 4
THRUST_TRANSITION_CMD = 600
GF_2_NEWTON = 0.001*9.81
NEWTON_2_GF = 1/GF_2_NEWTON


## DATAPATHS
dataPath = f"{BASE_PATH}/sar_projects/System_Identification/Motor_Impulse_Micro/Logs/"
fileName = f"StepsTestV2_2023-12-17_162536.csv"
filePath = os.path.join(dataPath,fileName)

## DEFINE FITTING FUNCTIONS AND SYSTEM CONSTANTS
def linear_func(x,a,b):
    return a*x + b

def quadratic_func(x,a):
    return  a*x**2

def sqrt_func(x,a,b):
    return  a*x**0.5 + b

def piecewise_func(x, a, b):
    """
    Piecewise function that is quadratic for x < x0 and linear for x >= x0.
    Parameters:
    - x0: The breakpoint between quadratic and linear
    - a, b, c: Parameters for the quadratic part (ax^2 + bx + c)
    - d: Slope for the linear part (dx)
    """
    x0 = THRUST_TRANSITION_CMD
    return np.piecewise(x, [x < x0, x >= x0],
                        [lambda x: a * x**2, 
                         lambda x: b * x + (a * x0**2 - b * x0)])

def piecewise_func_inv(x, a, b):
    x0 = THRUST_TRANSITION_CMD
    y0 = piecewise_func(x0, a, b)
    return np.piecewise(x, [x < y0, y0 <= x],
                        [lambda x: np.sqrt(x/a), 
                         lambda x: (x - (a * x0**2 - b * x0))/b])


## LOAD DATA
df = pd.read_csv(filePath,comment="#")
# df["Thrust (gf)"]+=2.4
df = df.query("`Voltage (V)` >= 3.90*@NUM_BATT_CELLS")


## INITIALIZE PLOTS
fig = plt.figure(figsize=(14,10),constrained_layout=False)
ax_Thrust_Constant = fig.add_subplot(2, 3, 1)
ax_ESC_Cmd = fig.add_subplot(2, 3, 2)
ax_Thrust_Cmd = fig.add_subplot(2, 3, 5)
ax_Torque_Constant = fig.add_subplot(2, 3, 4)
ax_ThrustTorque = fig.add_subplot(2, 3, 3)
ax_current = fig.add_subplot(2, 3, 6)


################# THRUST PLOT #################
thrust_data = df[["Thrust (gf)","Motor Optical Speed (rad/s)","Voltage (V)"]].dropna()
ax_Thrust_Constant.scatter(thrust_data["Motor Optical Speed (rad/s)"],thrust_data["Thrust (gf)"]*GF_2_NEWTON,c=thrust_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)

## FIT THRUST CONSTANT
params, _ = curve_fit(
        quadratic_func,
        xdata=thrust_data["Motor Optical Speed (rad/s)"],
        ydata=thrust_data["Thrust (gf)"]*GF_2_NEWTON,
        maxfev=5000,
    )
k_f = params[0]
temp_vals = np.linspace(0,thrust_data["Motor Optical Speed (rad/s)"].max(),100)
ax_Thrust_Constant.plot(temp_vals,quadratic_func(temp_vals,*params),color="grey",linestyle="--",label=r"Quadratic Fit: $F_t = k_f \cdot \omega^2$",zorder=0)

ax_Thrust_Constant.set_title("Thrust vs Angular Speed\n" f"$k_f = {k_f:.3e}$ [N/(rad/s)]")
ax_Thrust_Constant.set_xlabel("Angular Speed [rad/s]")
ax_Thrust_Constant.set_ylabel("Thrust [N]")
ax_Thrust_Constant.grid(True)
ax_Thrust_Constant.legend(loc="upper left")




################# TORQUE PLOT #################
torque_data = df[["Torque (N·m)","Motor Optical Speed (rad/s)","Voltage (V)"]].dropna()
ax_Torque_Constant.scatter(torque_data["Motor Optical Speed (rad/s)"],torque_data["Torque (N·m)"],c=torque_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)

## FIT TORQUE CONSTANT
params, _ = curve_fit(
        quadratic_func,
        xdata=torque_data["Motor Optical Speed (rad/s)"],
        ydata=torque_data["Torque (N·m)"],
        maxfev=5000,
    )
k_t = params[0]
temp_vals = np.linspace(0,torque_data["Motor Optical Speed (rad/s)"].max(),100)
ax_Torque_Constant.plot(temp_vals,quadratic_func(temp_vals,*params),color="grey",linestyle="--",label=r"Quadratic Fit: $T_t = k_t \cdot \omega^2$",zorder=0)


ax_Torque_Constant.set_title("Torque vs Angular Speed\n" f"$k_t = {k_t:.3e}$ [N·m/(rad/s)]")
ax_Torque_Constant.set_xlabel("Angular Speed [rad/s]")
ax_Torque_Constant.set_ylabel("Torque (N·m)")
ax_Torque_Constant.grid(True)
ax_Torque_Constant.legend(loc="upper left")



################# ESC CMD PLOT #################
ESC_data = df[["Thrust (gf)","ESC signal","Voltage (V)"]].dropna()
ax_ESC_Cmd.scatter(ESC_data["ESC signal"],ESC_data["Thrust (gf)"],c=ESC_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)

## FIT THRUST TO ESC CMD
params, _ = curve_fit(
        piecewise_func,
        xdata=ESC_data["ESC signal"].to_numpy(),
        ydata=ESC_data["Thrust (gf)"].to_numpy(),
        maxfev=5000,
    )

temp_vals = np.linspace(0,ESC_data["ESC signal"].max(),100)
ax_ESC_Cmd.plot(temp_vals,piecewise_func(temp_vals,*params),color="grey",linestyle="--",label="Quad & Linear Fit",zorder=0)

ax_ESC_Cmd.set_title("Thrust vs ESC Cmd\n" f"Params = {np.array2string(params, precision=2)}")
ax_ESC_Cmd.set_ylabel("Thrust (gf)")
ax_ESC_Cmd.set_xlabel("ESC Cmd")
ax_ESC_Cmd.set_xlim(-200,2200)
ax_ESC_Cmd.set_ylim(0,180)
ax_ESC_Cmd.vlines(2047,0,ESC_data["Thrust (gf)"].max(),color="black",linestyles='--',label="ESC Max")
ax_ESC_Cmd.grid(True)
ax_ESC_Cmd.legend(loc="upper left")



################# THRUST COMMAND PLOT #################
ESC_data = df[["Thrust (gf)","ESC signal","Voltage (V)"]].dropna()
ax_Thrust_Cmd.scatter(ESC_data["Thrust (gf)"],ESC_data["ESC signal"],c=ESC_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)

temp_vals = np.linspace(1,ESC_data["Thrust (gf)"].max(),100)
ax_Thrust_Cmd.plot(temp_vals,piecewise_func_inv(temp_vals,*params),color="grey",linestyle="--",label="Inverse Fit",zorder=0)

ax_Thrust_Cmd.set_title("ESC Cmd vs Thrust")
ax_Thrust_Cmd.set_xlabel("Thrust (gf)")
ax_Thrust_Cmd.set_ylabel("ESC Cmd")
ax_Thrust_Cmd.set_ylim(-200,2200)
ax_Thrust_Cmd.set_xlim(0,180)
ax_Thrust_Cmd.hlines(2047,0,ESC_data["Thrust (gf)"].max(),color="black",linestyles='--',label="ESC Max")
ax_Thrust_Cmd.grid(True)
ax_Thrust_Cmd.legend(loc="upper left")




################# TORQUE-THRUST PLOT #################
TorqueThrust_data = df[["Thrust (gf)","Torque (N·m)","Voltage (V)"]].dropna()
ax_ThrustTorque.scatter(TorqueThrust_data["Thrust (gf)"]*GF_2_NEWTON,TorqueThrust_data["Torque (N·m)"],c=TorqueThrust_data["Voltage (V)"]/NUM_BATT_CELLS,s=15)

## FIT THRUST TO TORQUE
params, _ = curve_fit(
        linear_func,
        xdata=TorqueThrust_data["Thrust (gf)"]*GF_2_NEWTON,
        ydata=TorqueThrust_data["Torque (N·m)"],
        maxfev=5000,
    )
C_TF = params[0]
temp_vals = np.linspace(0,TorqueThrust_data["Thrust (gf)"].max()*GF_2_NEWTON,100)
ax_ThrustTorque.plot(temp_vals,linear_func(temp_vals,*params),color="grey",linestyle="--",label=r"Linear Fit: $T_t = C_{TF} \cdot F_t$",zorder=0)


ax_ThrustTorque.set_title("Torque vs Thrust\n" f"$C_{{TF}} = {C_TF:.3e}$ [N/N·m]")
ax_ThrustTorque.set_xlabel("Thrust [N]")
ax_ThrustTorque.set_ylabel("Torque [N·m]")
ax_ThrustTorque.grid(True)
ax_ThrustTorque.legend()




## CURRENT PLOT
current_data = df[["Time (s)","Current (A)","Voltage (V)"]].dropna()
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
fig.savefig(f"{BASE_PATH}/sar_projects/System_Identification/Motor_Impulse_Micro/Plots/{fileName[:-4]}.png")
plt.show(block=True)

