import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize


# Motor calcs sourced from: https://github.com/PX4/sitl_gazebo/issues/110
# Values sourced from: https://wiki.bitcraze.io/misc:investigations:thrust
amps = np.array([0.24,0.37,0.56,0.75,0.94,1.15,1.37,1.59,1.83,2.11,2.39,2.71,3.06,3.46,3.88,4.44])
thrust_g = np.array([0,1.6,4.8,7.9,10.9,13.9,17.3,21,24.4,28.6,32.8,37.3,41.7,46,51.9,57.9])
V = np.array([4.01,3.98,3.95,3.92,3.88,3.84,3.8,3.76,3.71,3.67,3.65,3.62,3.56,3.48,3.4,3.3])
PWM_percent = np.array([0,6.25,12.5,18.75,25,31.25,37.5,43.25,50,56.25,62.5,68.75,75,81.25,87.5,93.75])
omega_rpm = np.array([0,4485,7570,9374,10885,12277,13522,14691,15924,17174,18179,19397,20539,21692,22598,23882])

thrust_N = 9.8/1000*thrust_g
omega_rad = omega_rpm *2*np.pi/60

Kv = 14000 # rpm/V [https://www.seeedstudio.com/Crazyflie-2-0-Spare-7x16-mm-coreless-DC-motor-with-connector-p-2115.html]
V_Rated = 4.2 # V
amps_Rated = 1.0 # Amps



## How is the max current 1.0 amps but the test would draw over 4.4 amps?


#region Motor Caclulations
# Max Rotational Velocity = Kv * Max Applied Voltage * Max Motor Efficiency * 2π / 60 []
# Motor Constant = Thrust / (Ω) ²
# Moment Constant = 60 / (2π * Kv)
# Rotor Drag Coefficient = Thrust / (ρ * (Kv * Max Applied Voltage * Max Motor Efficiency / 60) ² * Propeller diameter ⁴)

# where,
# Kv [RPM/V]
# Max Thrust [N]
# Max Applied Voltage [V]
# Ω Rotor angular velocity [rad/s]
# ρ Air density at 20ºC: 1.2041 [kg/m³]
#endregion

plt.figure(2)
plt.plot(V,thrust_g) # This graph doesn't match their graph



## Fit thrust to omega for k_f constant
plt.figure(1)
y_data = thrust_N
x_data = omega_rad
def test_func(x, a):
    return a *x*x

params, params_covariance = optimize.curve_fit(test_func, x_data, y_data)
print(params/4)

plt.scatter(x_data, y_data, label='Data')
plt.plot(x_data, test_func(x_data, params[0]),
         label='Fitted function')

plt.legend(loc='best')
plt.xlabel("Omega rad/s")
plt.ylabel("Thrust N")
plt.grid(True)
plt.show()