import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

## SYSTEM PARAMETERS
tau = 0.05
t_max = 3

## DEFINE INPUT FUNCTION
def u(t):
    if t < 0.5:
        t = t

    elif t > 2.0:
        t = 0
    else:
        t = 1.0

    return t

## DEFINE ODE
def f(t,x,c):
    dxdt = -1/tau*x + 1/tau*u(t)

    return dxdt

## DEFINE TIME SPANS, INITIAL VALUES, AND CONSTANTS
tspan = np.linspace(0,t_max,1000)
x_init = [0]
c = []

## SOLVE ODE
sol = solve_ivp(lambda t,x: f(t,x,c), [tspan[0],tspan[-1]], x_init, t_eval=tspan,rtol = 1e-5)


## FILTER PARAMETERS
dt = 0.01
alpha = np.exp(-dt/tau)


## FILTER FUNCTION
dt_list = np.arange(0,t_max,dt)
u_k = [u(t) for t in dt_list]
X_k = np.zeros_like(dt_list)

for ii,k in enumerate(dt_list):

    if ii == 0:
        X_k[ii] = 0
    else:
        X_k[ii] = X_k[ii-1]*alpha + (1-alpha)*u_k[ii]


## PLOT FILTER RESPONSE
fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(sol.t,[u(t) for t in sol.t],label="X_desired",linestyle="--",color='grey')
ax.plot(sol.t,sol.y[0],label="X_continuous")
ax.plot(dt_list,X_k,label="X_discrete",linestyle="--")

ax.set_xlabel("Time")
ax.set_ylabel("State")
ax.set_title("First order filter response")

ax.grid()
ax.legend()

plt.show()

# while(True):

#     pass