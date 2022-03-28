import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

## SYSTEM PARAMETERS
tau = 0.1
X_0 = 1
t_max = 10

def func(t_list):
    
    return [0 if x < 5.0 else 2.0 for x in t_list]


def u(t):
    if t < 5.0:
        t = 0

    else:
        t = 2.0

    return t


## FILTER PARAMETERS

dt = 0.001
alpha = np.exp(-dt/tau)

## CREATE ARRAY FOR DESIRED STATE
t = np.linspace(0,t_max,100)
X_d = func(t)
X_d[0] = 0


## CONTINOUS TIME SYSTEM
X_t = X_d*(1-np.exp(-t/tau))




dt_list = np.arange(0,t_max,dt)
u_k = func(dt_list)
X_k = np.zeros_like(dt_list)

for ii,k in enumerate(dt_list):

    if ii == 0:
        X_k[ii] = 0
    else:
        X_k[ii] = X_k[ii-1]*alpha + (1-alpha)*u_k[ii]


fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(t,X_t,label="X_continuous")
ax.plot(t,X_d,label="X_desired",linestyle="--")
ax.plot(dt_list,X_k,label="X_filter",linestyle="--")

ax.set_xlabel("Time")
ax.set_ylabel("State")
ax.set_title("First order filter response")

ax.grid()
ax.legend()

plt.show()