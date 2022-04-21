import matplotlib.pyplot as plt
import numpy as np

## GOAL: FIND TRAJECTORY STARTING POSITIONS THAT ALLOW FOR DESIRED VELOCITIES TO BE REACHED


## FLIGHT CONDITIONS
V = 3.0     # [m/s]
phi = 20    # [deg]
h_c = 2.1

## DESIRED IMPACT LOCATION
X_f = 5.0
Z_f = h_c
d_min = 0.2

phi_rad = np.radians(phi)
Vx = V*np.cos(phi_rad)
Vz = V*np.sin(phi_rad)


## SYSTEM ACCELERATIONS
a_z = 3.1   # [m/s^2]
a_x = 1.0   # [m/s^2]


t_x = Vx/a_x   # Time Vx reached
t_z = Vz/a_z   # Time Vz reached
t_arr = np.linspace(0,5,500)

z_0_guess = 0.4
x_0_guess = 0

def calcTraj(t,x_0,z_0,v_x,v_z):

    if t < t_x:
        x_t = 0.5*a_x*t**2 + x_0
        Vx_t = a_x*t

        z_t = z_0
        Vz_t = 0

    ## VERTICAL ACCELERATION
    elif t_x <= t < t_x + t_z:
        x_t = v_x*t - v_x**2/(2*a_x) + x_0
        Vx_t = v_x

        z_t = 0.5*a_z*(t-t_x)**2 + z_0
        Vz_t = a_z*(t-t_x)

    ## VELOCITY COASTING
    elif t_x + t_z <= t:
        x_t = v_x*t - v_x**2/(2*a_x) + x_0
        Vx_t = v_x

        z_t = v_z*(t-t_x) - v_z**2/(2*a_z) + z_0
        Vz_t = v_z

    return [x_t,Vx_t,z_t,Vz_t]


## CALCULATE ORIGINAL TRAJECTORY
x_t = []
z_t = []
Vx_t = []
Vz_t = []
for ii,t in enumerate(t_arr):

    X = calcTraj(t,x_0_guess,z_0_guess,Vx,Vz)

    if X[2] >= h_c:
        t_arr = t_arr[:ii]
        break

    x_t.append(X[0])
    Vx_t.append(X[1])
    z_t.append(X[2])
    Vz_t.append(X[3])

_,_,z_vz,_ = calcTraj(t_x+t_z,x_0_guess,z_0_guess,Vx,Vz)
z_0 = z_0_guess + (h_c - d_min - z_vz)


## CALCULATE SHIFTED LEFT TRAJECTORY
x_impact = x_t[-1]
x_0 = x_0_guess + X_f - x_t[-1] + (z_0 - z_0_guess)*Vx/Vz




fig = plt.figure()
ax = fig.add_subplot(211)
ax.plot(np.array(x_t)+(x_0-x_0_guess),np.array(z_t)+(z_0 - z_0_guess))



# ax.scatter(x_f,z_val,label='Vx reached')
# ax.scatter(x_val,z_f,label='Vz reached')


ax.axhline(y=h_c,color='k',linestyle='--',label='h_ceiling')
ax.axhline(y=h_c-d_min,color='k',linestyle='--',alpha=0.5,label='d_min')
ax.set_ylim(0,h_c + 0.5)
ax.set_xlim(-10,10)

ax.set_xlabel('X_Pos [m]')
ax.set_ylabel('Z_Pos [m]')
ax.grid()
ax.legend(loc='upper left')

ax2 = fig.add_subplot(212)
ax2.plot(t_arr,Vz_t,label='V_z')
ax2.plot(t_arr,Vx_t,label='V_x')

ax2.set_xlabel('time [s]')
ax2.set_ylabel('Vel [m]')
ax2.grid()
ax2.legend(loc='upper left')


fig.tight_layout()
plt.show()




