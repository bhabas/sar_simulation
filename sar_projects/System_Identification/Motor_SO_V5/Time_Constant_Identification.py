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
fileName = f"Log_2023-12-11_145618.csv"
filePath = os.path.join(dataPath,fileName)

## DEFINE FITTING FUNCTIONS AND SYSTEM CONSTANTS
def exp_decay(t,a,tau,SS_value):
    return  np.exp(-(t-a)/tau) + SS_value

def exp_rise(t,a,tau,SS_value):
    return -np.exp(-(t-a)/tau) + SS_value

## LOAD DATA
df = pd.read_csv(filePath)
t = df['Time (s)'].to_numpy()
thrust = df["Thrust (gf)"].to_numpy()
torque = df["Torque (N·m)"].to_numpy()
angular_speed = df["Motor Optical Speed (rad/s)"].to_numpy()
voltage = df["Voltage (V)"].to_numpy()
curent = df["Current (A)"].to_numpy()
ESC_signal = df["ESC signal"].to_numpy()/2047


## INITIALIZE PLOTS
fig = plt.figure(figsize=(14,10),constrained_layout=False)
ax_thrust = fig.add_subplot(2, 3, 1)
ax_ESC_signal = fig.add_subplot(2, 3, 2)
ax_torque = fig.add_subplot(2, 3, 3)
ax_angular_speed = fig.add_subplot(2, 3, 4)
ax_voltage = fig.add_subplot(2, 3, 5)
ax_current = fig.add_subplot(2, 3, 6)


## THRUST PLOT
ax_thrust.plot(t,thrust,label="Raw Data")
ax_thrust.set_title("Thrust")
ax_thrust.set_xlabel("Time [s]")
ax_thrust.set_ylabel("Thrust [gf]")
ax_thrust.grid(True)
ax_thrust.legend()

## ESC CMD PLOT
ax_ESC_signal.plot(t,ESC_signal,label="ESC Cmd")
ax_ESC_signal.set_title("ESC Signal")
ax_ESC_signal.set_xlabel("Time [s]")
ax_ESC_signal.set_title("ESC Cmd")
ax_ESC_signal.set_ylim(0,1.1)
ax_ESC_signal.grid(True)
ax_ESC_signal.legend()

## TORQUE PLOT
ax_torque.plot(t,torque,label="Raw Data")
ax_torque.set_title("Torque")
ax_torque.set_xlabel("Time [s]")
ax_torque.set_ylabel("Torque [N*m]")
ax_torque.grid(True)
ax_torque.legend()

## ANGULAR SPEED PLOT
ax_angular_speed.plot(t,angular_speed,label="Raw Data")
ax_angular_speed.set_title("Angular Speed")
ax_angular_speed.set_xlabel("Time [s]")
ax_angular_speed.set_ylabel("Angular Speed [rad/s]")
ax_angular_speed.grid(True)
ax_angular_speed.legend()

## VOLTAGE PLOT
ax_voltage.plot(t,voltage,label="Raw Data")
ax_voltage.set_title("Voltage")
ax_voltage.set_xlabel("Time [s]")
ax_voltage.set_ylabel("Voltage [V]")
ax_voltage.grid(True)
ax_voltage.legend()

## CURRENT PLOT
ax_current.plot(t,curent,label="Raw Data")
ax_current.set_title("Current")
ax_current.set_xlabel("Time [s]")
ax_current.set_ylabel("Current [A]")
ax_current.grid(True)
ax_current.legend()


## CURVE FITS
t_ranges = [[3.925,4.5],[5.95,6.1],[7.88,8.15]]
exp_funcs = [exp_rise,exp_rise,exp_decay]
init_params = [[4.02,0.02,600],[4.02,0.02,600],[8,0.02,540]]

# for t_range,exp_func,p0 in zip(t_ranges,exp_funcs,init_params):

#     mask = (t_range[0] < t) & (t <= t_range[1])

#     t_fit = t[mask]
#     thrust_fit = thrust[mask]

#     params,_ = curve_fit(
#             exp_func,
#             xdata=t,
#             ydata=thrust,
#             maxfev=5000,
#             p0=p0)

#     fitted_curve = exp_func(t_fit,*params)
#     ax_angular_speed.plot(t_fit,fitted_curve,'--',color='r',alpha=0.9,label="Curve Fit")

test_range = np.linspace(7.88,8.15,100)
test_curve = exp_decay(test_range,8,0.02,540)
ax_angular_speed.plot(test_range,test_curve,'--',color='k',alpha=0.9,label="Test Curve Fit")
ax_angular_speed.legend()

fig.tight_layout()
# fig.suptitle(f"Data from {fileName}")
# fig.savefig(f"{BASE_PATH}/sar_projects/System_Identification/Motor_SO_V5/Plots/{fileName[:-4]}.png")
plt.show(block=True)

