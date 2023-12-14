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

## LOAD DATA
df = pd.read_csv(filePath,comment="#")
# df = df.query("`Voltage (V)` >= 3.85*6")


## INITIALIZE PLOTS
fig = plt.figure(figsize=(14,10),constrained_layout=False)
ax_thrust = fig.add_subplot(2, 3, 1)
ax_ESC_signal = fig.add_subplot(2, 3, 2)
ax_voltage = fig.add_subplot(2, 3, 3)
ax_angular_speed = fig.add_subplot(2, 3, 4)
ax_torque = fig.add_subplot(2, 3, 5)
ax_current = fig.add_subplot(2, 3, 6)


## THRUST PLOT
thrust_data = df[["Thrust (gf)","Motor Optical Speed (rad/s)","Voltage (V)"]].interpolate()
ax_thrust.scatter(thrust_data["Motor Optical Speed (rad/s)"].to_numpy(),thrust_data["Thrust (gf)"].to_numpy(),c=thrust_data["Voltage (V)"].to_numpy()/NUM_BATT_CELLS,s=15)
ax_thrust.set_title("Thrust vs Angular Speed")
ax_thrust.set_xlabel("Angular Speed [rad/s]")
ax_thrust.set_ylabel("Thrust [gf]")
ax_thrust.grid(True)
# ax_thrust.legend()

## ESC CMD PLOT
ESC_data = df[["Thrust (gf)","ESC signal","Voltage (V)"]]
ax_ESC_signal.scatter(ESC_data["Thrust (gf)"].to_numpy(),ESC_data["ESC signal"].to_numpy(),c=ESC_data["Voltage (V)"].to_numpy()/NUM_BATT_CELLS,s=15)
ax_ESC_signal.set_title("ESC Cmd vs Thrust")
ax_ESC_signal.set_xlabel("Thrust (gf)")
ax_ESC_signal.set_ylabel("ESC Cmd")
ax_ESC_signal.set_ylim(-200,2200)
ax_ESC_signal.hlines(2047,0,ESC_data["Thrust (gf)"].to_numpy().max(),color="black",linestyles='--',label="ESC Max")
ax_ESC_signal.grid(True)
ax_ESC_signal.legend(loc="lower right")

## TORQUE PLOT  
torque_data = df[["Thrust (gf)","Torque (N·m)","Voltage (V)"]].interpolate()
ax_torque.scatter(torque_data["Torque (N·m)"].to_numpy(),torque_data["Thrust (gf)"].to_numpy(),c=torque_data["Voltage (V)"].to_numpy()/NUM_BATT_CELLS,s=15)
ax_torque.set_title("Thrust vs Torque")
ax_torque.set_xlabel("Torque [N·m]")
ax_torque.set_ylabel("Thrust [gf]")
ax_torque.grid(True)
# ax_torque.legend()


## ANGULAR SPEED PLOT
angular_speed_data = df[["Torque (N·m)","Motor Optical Speed (rad/s)","Voltage (V)"]].interpolate()
ax_angular_speed.scatter(angular_speed_data["Motor Optical Speed (rad/s)"].to_numpy(),angular_speed_data["Torque (N·m)"].to_numpy(),c=angular_speed_data["Voltage (V)"].to_numpy()/NUM_BATT_CELLS,s=15)
ax_angular_speed.set_title("Torque vs Angular Speed")
ax_angular_speed.set_xlabel("Angular Speed [rad/s]")
ax_angular_speed.set_ylabel("Torque (N·m)")
ax_angular_speed.grid(True)


## VOLTAGE PLOT
voltage_data = df[["Thrust (gf)","Voltage (V)"]].interpolate()
ax_voltage.scatter(voltage_data["Thrust (gf)"].to_numpy(),voltage_data["Voltage (V)"].to_numpy()/NUM_BATT_CELLS,c=voltage_data["Voltage (V)"].to_numpy()/NUM_BATT_CELLS,s=15)
ax_voltage.set_title("Thrust vs Voltage")
ax_voltage.set_ylabel("Voltage [V]")
ax_voltage.set_xlabel("Thrust [gf]")
ax_voltage.set_ylim(3.6,4.5)
ax_voltage.set(xlim=(0, voltage_data["Thrust (gf)"].to_numpy().max()))
ax_voltage.hlines(4.2,0,5E3,color="black",linestyles='--',label="Voltage Max")
ax_voltage.hlines(3.3,0,5E3,color="black",linestyles='--',label="Voltage Min")
ax_voltage.grid(True)
# ax_voltage.legend()

## CURRENT PLOT
current_data = df[["Time (s)","Current (A)","Voltage (V)"]].interpolate()
ax_current.scatter(current_data["Time (s)"].to_numpy(),current_data["Current (A)"].to_numpy(),c=voltage_data["Voltage (V)"].to_numpy()/NUM_BATT_CELLS,s=15)
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

