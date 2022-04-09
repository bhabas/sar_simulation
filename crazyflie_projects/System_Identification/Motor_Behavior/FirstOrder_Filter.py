'''
This script validates the behavior between a discrete 1st-order filter w/ arbitrary 
input function and a first order system with arbitrary input function. This is used
in the validation for the motor model implemented in Gazebo to model the lagging
behavior of realistic motors speeding up and slowing down since instantaneous speed
is not possible.
Date: 3/28/2022
Author: Bryan Habas
'''

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


## SYSTEM PARAMETERS
tau_up = 0.025  # Motor speed up time constant
tau_down = 0.1  # Motor slow down time constant
t_max = 3       # Max time tested

## DEFINE INPUT FUNCTION
def u(t):
    if t < 0.5:
        t = t

    elif 0.5 <= t < 2.0:
        t = 1.0

    else:
        t = 0.0

    return t

## DEFINE ODE
def f(t,x):
    
    if u(t) > x:
        dxdt = -1/tau_up*x + 1/tau_up*u(t)
    else:
        dxdt = -1/tau_down*x + 1/tau_down*u(t)
    return dxdt

## DEFINE TIME SPANS, INITIAL VALUES, AND CONSTANTS
tspan = np.linspace(0,t_max,1000)
x_init = [0]


## SOLVE ODE
sol = solve_ivp(lambda t,x: f(t,x), [tspan[0],tspan[-1]], x_init, t_eval=tspan,rtol = 1e-5)


## FILTER PARAMETERS
dt = 0.001
alpha_up = np.exp(-dt/tau_up)
alpha_down = np.exp(-dt/tau_down)


## FILTER FUNCTION
dt_list = np.arange(0,t_max,dt)
u_k = [u(t) for t in dt_list]
x_k = np.zeros_like(dt_list)

for ii,k in enumerate(dt_list):

    if ii == 0:
        x_k[ii] = 0
    else:
        if u_k[ii] > x_k[ii]:
            x_k[ii] = x_k[ii-1]*alpha_up + (1-alpha_up)*u_k[ii]

        else:
            x_k[ii] = x_k[ii-1]*alpha_down + (1-alpha_down)*u_k[ii]


## PLOT FILTER RESPONSE
fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(sol.t,[u(t) for t in sol.t],label="X_desired",linestyle="--",color='grey')
ax.plot(sol.t,sol.y[0],label="X_continuous")
ax.plot(dt_list,x_k,label="X_discrete",linestyle="--")

ax.set_xlabel("Time")
ax.set_ylabel("State")
ax.set_title("First order filter response")

ax.grid()
ax.legend()

plt.show()
