import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import os


import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_projects'))


RPM_TO_RAD_PER_SEC = 2*np.pi/60

## DATAPATHS
dataPath = f"{BASE_PATH}/sar_projects/System_Identification/Motor_SO_V5/Logs/"
fileName = f"StepsTestV2_2023-12-12_102702.csv"
filePath = os.path.join(dataPath,fileName)

## LOAD DATA
df = pd.read_csv(filePath)


## INITIALIZE PLOTS
fig = plt.figure(figsize=(14,10),constrained_layout=False)
ax_thrust = fig.add_subplot(2, 3, 1)
ax_ESC_signal = fig.add_subplot(2, 3, 2)
ax_voltage = fig.add_subplot(2, 3, 3)
ax_torque = fig.add_subplot(2, 3, 4)
ax_angular_speed = fig.add_subplot(2, 3, 5)
ax_current = fig.add_subplot(2, 3, 6)


## THRUST PLOT
thrust_data = df[['Time (s)',"Thrust (gf)"]].interpolate()
ax_thrust.plot(thrust_data["Time (s)"].to_numpy(),thrust_data["Thrust (gf)"].to_numpy())
ax_thrust.set_title("Thrust")
ax_thrust.set_xlabel("Time [s]")
ax_thrust.set_ylabel("Thrust [gf]")
ax_thrust.grid(True)
# ax_thrust.legend()

## ESC CMD PLOT
ESC_data = df[['Time (s)',"ESC signal"]]
ax_ESC_signal.plot(ESC_data["Time (s)"].to_numpy(),ESC_data["ESC signal"].to_numpy())
ax_ESC_signal.set_title("ESC Signal")
ax_ESC_signal.set_xlabel("Time [s]")
ax_ESC_signal.set_title("ESC Cmd")
ax_ESC_signal.set_ylim(-200,2200)
ax_ESC_signal.hlines(2047,0,t[-1],color="black",linestyles='--',label="ESC Max")
ax_ESC_signal.grid(True)
ax_ESC_signal.legend(loc="lower right")

## TORQUE PLOT
Torque_data = df[['Time (s)',"Torque (N·m)"]].interpolate()
ax_torque.plot(Torque_data["Time (s)"].to_numpy(),Torque_data["Torque (N·m)"].to_numpy())
ax_torque.set_title("Torque")
ax_torque.set_xlabel("Time [s]")
ax_torque.set_ylabel("Torque [N*m]")
ax_torque.grid(True)
# ax_torque.legend()

## ANGULAR SPEED PLOT
angular_speed_data = df[['Time (s)',"Motor Optical Speed (rad/s)"]].interpolate()
ax_angular_speed.plot(angular_speed_data["Time (s)"].to_numpy(),angular_speed_data["Motor Optical Speed (rad/s)"].to_numpy())
ax_angular_speed.scatter(angular_speed_data["Time (s)"].to_numpy(),angular_speed_data["Motor Optical Speed (rad/s)"].to_numpy(),s=3)

ax_angular_speed.set_title("Angular Speed")
ax_angular_speed.set_xlabel("Time [s]")
ax_angular_speed.set_ylabel("Angular Speed [rad/s]")
ax_angular_speed.grid(True)
# ax_angular_speed.set_xlim(23,23.5)
# ax_angular_speed.legend()

## VOLTAGE PLOT
voltage_data = df[['Time (s)',"Voltage (V)"]].interpolate()
ax_voltage.plot(voltage_data["Time (s)"].to_numpy(),voltage_data["Voltage (V)"].to_numpy()/NUM_BATT_CELLS)
ax_voltage.set_title("Voltage")
ax_voltage.set_xlabel("Time [s]")
ax_voltage.set_ylabel("Voltage [V]")
ax_voltage.set_ylim(3.0,4.5)
ax_voltage.hlines(4.2,0,t[-1],color="black",linestyles='--',label="Voltage Max")
ax_voltage.hlines(3.3,0,t[-1],color="black",linestyles='--',label="Voltage Min")
ax_voltage.grid(True)
# ax_voltage.legend()

## CURRENT PLOT
current_data = df[['Time (s)',"Current (A)"]].interpolate()
ax_current.plot(current_data["Time (s)"].to_numpy(),current_data["Current (A)"].to_numpy())
ax_current.set_title("Current")
ax_current.set_xlabel("Time [s]")
ax_current.set_ylabel("Current [A]")
ax_current.set_ylim(0)
ax_current.grid(True)
# ax_current.legend()

fig.tight_layout()
# fig.suptitle(f"Data from {fileName}")
# fig.savefig(f"{BASE_PATH}/sar_projects/System_Identification/Motor_SO_V5/Plots/{fileName[:-4]}.png")
plt.show(block=True)

