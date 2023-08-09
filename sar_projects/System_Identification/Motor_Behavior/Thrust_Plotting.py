import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import os

## ADD SAR_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(0,BASE_PATH)
from crazyflie_logging.data_analysis.Data_Analysis import DataFile

## DATAPATHS
fileName = f"MS_Decay_3.csv"
dataPath = f"{BASE_PATH}/crazyflie_projects/System_Identification/Motor_Behavior/logs/Upgraded_Motor_Decay"
filePath = os.path.join(dataPath,fileName)



## DEFINE FITTING FUNCTIONS AND SYSTEM CONSTANTS
def exp_decay(x,a,b,c):
    return np.exp(-(x-a)/b) + c

def exp_rise(x,a,b,c):
    return -np.exp(-(x-a)/b) + c

k_f = 2.2e-8    # Thrust Coefficient [N/(rad/s)^2]
g2Newton = 1000/9.81



## LOAD DATA
df = pd.read_csv(filePath)
t_raw = df['time'].to_numpy()
thrust_raw = df["omega"].to_numpy()**2*k_f*g2Newton

## FIT CURVE TO SPECIFIED RANGE
t_min = 5.76
t_max = 8

df_fit = df.query(f"{t_min} <= time < {t_max}")
t_fit = df_fit['time'].to_numpy()
thrust_fit = df_fit["omega"].to_numpy()**2*k_f*g2Newton

(a,b,c),_ = curve_fit(
        exp_rise,
        xdata=t_fit,
        ydata=thrust_fit,
        p0=[25.5,0.25,5.25])
print(f"a: {a:.3f} | b: {b:.3f} | c: {c:.3f}")


## GENERATE FIGURE
fig = plt.figure(figsize=(6,3))
ax = fig.add_subplot(111)

t_arr = np.linspace(t_min,t_max,100)

raw_data = ax.plot(t_raw,thrust_raw,alpha=0.7,label="Recorded Thrust",color="tab:blue")
# fit_data = ax.plot(t_arr,exp_rise(t_arr,a,b,c),'--',color='k',alpha=1.0,label='Thrust Predicted')
# custom_fit = ax.plot(t_arr,exp_decay(t_arr,25.5,0.25,5.25),'r--',alpha=0.8)



## CURVE FITS FOR MS_Decay_3
ax.plot(np.linspace(5.8,8.0,100),exp_rise(np.linspace(5.8,8.0,100),6.0,0.144,5.27),'--',color='k',alpha=0.9,label="Curve Fit")
ax.plot(np.linspace(10.8,13.0,100),exp_rise(np.linspace(10.8,13.0,100),10.896,0.057,12.274),'--',color='k',alpha=0.9)
ax.plot(np.linspace(15.78,18.75,100),exp_decay(np.linspace(15.78,18.75,100),16.164,0.164,0.110),'--',color='k',alpha=0.9)



## FIGURE SETTINGS AND ANNOTATIONS
# ax.set_title(f"Thrust = e^(-(x-a)/b)+c --- a: {a:.3f} | b: {b:.3f} | c: {c:.3f}")
# ax.set_ylim(0,17)
ax.set_yticks([0,5,10,15])
ax.set_xlabel('Time [s]')
ax.set_ylabel('Thrust [g]')
ax.grid()
ax.legend()

fig.tight_layout()

plt.savefig("Thrust_Decay.pdf",dpi=300)
plt.show()