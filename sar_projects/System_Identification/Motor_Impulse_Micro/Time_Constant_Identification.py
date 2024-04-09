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
K_F = 1.521e-7

## DATAPATHS
dataPath = f"{BASE_PATH}/sar_projects/System_Identification/Motor_Impulse_Micro/Logs/"
fileName = f"TimeConstant_Test_2024-02-06_134515.csv"
filePath = os.path.join(dataPath,fileName)

## DEFINE FITTING FUNCTIONS AND SYSTEM CONSTANTS
def exp_decay(t,a,tau,SS_value):
    return  np.exp(-(t-a)/tau) + SS_value

def exp_rise(t,a,tau,SS_value):
    return -np.exp(-(t-a)/tau) + SS_value

## LOAD DATA
df = pd.read_csv(filePath,comment="#")


## INITIALIZE PLOTS
fig = plt.figure(figsize=(14,10),constrained_layout=False)
ax_thrust = fig.add_subplot(2, 3, 1)
ax_ESC_signal = fig.add_subplot(2, 3, 2)
ax_voltage = fig.add_subplot(2, 3, 3)
ax_norm_angular_speed = fig.add_subplot(2, 3, 4)
ax_angular_speed = fig.add_subplot(2, 3, 5)
ax_current = fig.add_subplot(2, 3, 6)


## THRUST PLOT
thrust_data = df[["Time (s)","Thrust (gf)","Motor Optical Speed (rad/s)"]].interpolate()
ax_thrust.plot(thrust_data["Time (s)"].to_numpy(),thrust_data["Thrust (gf)"].to_numpy())
ax_thrust.plot(thrust_data["Time (s)"].to_numpy(),K_F*(thrust_data["Motor Optical Speed (rad/s)"].to_numpy())**2*NEWTON_2_GF)

ax_thrust.set_title("Thrust")
ax_thrust.set_xlabel("Time [s]")
ax_thrust.set_ylabel("Thrust [gf]")
ax_thrust.grid(True)
# ax_thrust.legend()

## ESC CMD PLOT
ESC_data = df[["Time (s)","ESC signal"]]
ax_ESC_signal.plot(ESC_data["Time (s)"].to_numpy(),ESC_data["ESC signal"].to_numpy())
ax_ESC_signal.set_title("ESC Signal")
ax_ESC_signal.set_xlabel("Time [s]")
ax_ESC_signal.set_title("ESC Cmd")
ax_ESC_signal.set_ylim(-200,2200)
ax_ESC_signal.hlines(2047,0,ESC_data["Time (s)"].to_numpy()[-1],color="black",linestyles='--',label="ESC Max")
ax_ESC_signal.grid(True)
ax_ESC_signal.legend(loc="lower right")

## NORMALIZED ANGULAR SPEED PLOT
norm_angular_speed_data = df[["Time (s)","Motor Optical Speed (rad/s)"]].interpolate()
norm_angular_speed_data["Motor Optical Speed (rad/s)"] = norm_angular_speed_data["Motor Optical Speed (rad/s)"]/4000

ax_norm_angular_speed.plot(norm_angular_speed_data["Time (s)"].to_numpy(),norm_angular_speed_data["Motor Optical Speed (rad/s)"].to_numpy(),alpha=0.5)
ax_norm_angular_speed.scatter(norm_angular_speed_data["Time (s)"].to_numpy(),norm_angular_speed_data["Motor Optical Speed (rad/s)"].to_numpy(),alpha=0.5,s=5)
ax_norm_angular_speed.set_title("Normalized Angular Speed")
ax_norm_angular_speed.set_xlabel("Time [s]")
ax_norm_angular_speed.set_ylabel("Norm Angular Speed [rad/s]")
ax_norm_angular_speed.grid(True)


## ANGULAR SPEED PLOT
angular_speed_data = df[["Time (s)","Motor Optical Speed (rad/s)"]].interpolate()
angular_speed_data["Motor Optical Speed (rad/s)"] = angular_speed_data["Motor Optical Speed (rad/s)"]

ax_angular_speed.plot(angular_speed_data["Time (s)"].to_numpy(),angular_speed_data["Motor Optical Speed (rad/s)"].to_numpy())
ax_angular_speed.set_title("Angular Speed")
ax_angular_speed.set_xlabel("Time [s]")
ax_angular_speed.set_ylabel("Angular Speed [rad/s]")
ax_angular_speed.grid(True)


## VOLTAGE PLOT
voltage_data = df[["Time (s)","Voltage (V)"]].interpolate()
ax_voltage.plot(voltage_data["Time (s)"].to_numpy(),voltage_data["Voltage (V)"].to_numpy()/NUM_BATT_CELLS)
ax_voltage.set_title("Voltage")
ax_voltage.set_xlabel("Time [s]")
ax_voltage.set_ylabel("Voltage [V]")
ax_voltage.set_ylim(3.0,4.5)
ax_voltage.hlines(4.2,0,voltage_data["Time (s)"].to_numpy()[-1],color="black",linestyles='--',label="Voltage Max")
ax_voltage.hlines(3.3,0,voltage_data["Time (s)"].to_numpy()[-1],color="black",linestyles='--',label="Voltage Min")
ax_voltage.grid(True)
# ax_voltage.legend()

## CURRENT PLOT
current_data = df[["Time (s)","Current (A)"]].interpolate()
ax_current.plot(current_data["Time (s)"].to_numpy(),current_data["Current (A)"].to_numpy())
ax_current.set_title("Current")
ax_current.set_xlabel("Time [s]")
ax_current.set_ylabel("Current [A]")
ax_current.set_ylim(0)
ax_current.grid(True)
# ax_current.legend()


## CURVE FITS
curve_fits = {
    'a': (exp_rise, [2.57, 2.8]),
    'b': (exp_decay, [4.108, 4.5]),

    'c': (exp_decay, [8.346, 8.7]),
    'd': (exp_decay, [12.58, 13.0]),
    # 'e': (exp_rise, [11.59, 11.95]),
    # 'f': (exp_decay, [12.79, 13.2]),
    # 'g': (exp_decay, [14.53, 15.0]),
    # 'h': (exp_decay, [15.87, 16.3]),


    # 'i': (exp_rise, [18.34, 18.6]),
    # 'j': (exp_rise, [20.54, 20.8]),

    # 'k': (exp_decay, [22.63, 23.0]),
    # 'l': (exp_decay, [31.61, 32.6]),

    # 'm': (exp_rise, [34.14, 34.6]),
    # 'n': (exp_decay, [38.03, 38.4]),
    # 'o': (exp_decay, [39.77, 40.2]),
}


for key, (exp_func, t_range) in curve_fits.items():
    mask = (t_range[0] < norm_angular_speed_data["Time (s)"].to_numpy()) & (norm_angular_speed_data["Time (s)"].to_numpy() <= t_range[1])

    t_fit = norm_angular_speed_data["Time (s)"].to_numpy()[mask]
    thrust_fit = norm_angular_speed_data["Motor Optical Speed (rad/s)"].to_numpy()[mask]
    
    # Assuming the initial guess p0 needs to be set for each fitting
    p0 = [t_range[0], 0.03, 0]  # Modify this as per your requirement

    params, _ = curve_fit(
        exp_func,
        xdata=t_fit,
        ydata=thrust_fit,
        maxfev=5000,
        p0=p0
    )

    fitted_curve = exp_func(t_fit,*params)
    ax_norm_angular_speed.plot(t_fit,fitted_curve,'--',color='r',alpha=0.9,label="Curve Fit")

    print(f"{key}-> Type: {exp_func.__name__} \t Time-Constant: {params[1]:.3f} \t Params: {params}")

test_range = np.linspace(23,23.5,1000)
test_curve = exp_decay(test_range,23.25,0.031,5.3*400)
# ax_angular_speed.plot(test_range,test_curve,'--',color='k',alpha=0.9,label="Test Curve Fit")
# ax_angular_speed.legend()

fig.tight_layout()
fig.subplots_adjust(top=0.9)
fig.suptitle(f"Data from {fileName}")
fig.savefig(f"{BASE_PATH}/sar_projects/System_Identification/Motor_Impulse_Micro/Plots/{fileName[:-4]}.png")
plt.show(block=True)

