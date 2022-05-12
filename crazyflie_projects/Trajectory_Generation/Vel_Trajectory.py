import matplotlib.pyplot as plt
import numpy as np

## GOAL: FIND TRAJECTORY STARTING POSITIONS THAT ALLOW FOR DESIRED VELOCITIES TO BE REACHED

## SYSTEM CONSTRAINTS
H_CEIL = 2.1    # Height of ceiling [m]
a_x = 1.0       # X-acceleration [m/s^2]
a_z = 2.0       # Z-acceleration [m/s^2]

## DESIRED IMPACT LOCATION
x_impact = 2.0

## FLIGHT CONDITIONS
V = 3.0     # Flight vel [m/s]
phi = 30    # Flight angle [deg]
phi_rad = np.radians(phi) # [rad]
d_vel = 0.7     # Distance where (Vx,Vz) are reached [m]

Vx = V*np.cos(phi_rad) # [m/s]
Vz = V*np.sin(phi_rad) # [m/s]


t_x = Vx/a_x   # Time Vx reached
t_z = Vz/a_z   # Time Vz reached
t_arr = np.linspace(0,10,1000)



def calcTraj(t,x_0,z_0,v_x,v_z):
    """Returns trajectory values at a given time

    Args:
        t (float): Time along trajectory
        x_0 (float): Starting x-location
        z_0 (float): Starting z-location
        v_x (float): Desired x-vel
        v_z (float): Desired z-vel

    Returns:
        [x_t,Vx_t,z_t,Vz_t]: State values at given time
    """    

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


z_vz = 1/2*a_z*t_z**2
z_0 = H_CEIL - d_vel - z_vz

x_vz = Vx*(t_x+t_z) - Vx**2/(2*a_x)
x_0 = x_impact - x_vz - d_vel*Vx/Vz

## CALCULATE ORIGINAL TRAJECTORY
x_t = []
z_t = []
Vx_t = []
Vz_t = []


for ii,t in enumerate(t_arr):

    X = calcTraj(t,x_0,z_0,Vx,Vz)

    if X[2] >= H_CEIL:
        t_arr = t_arr[:ii]
        break

    x_t.append(X[0])
    Vx_t.append(X[1])
    z_t.append(X[2])
    Vz_t.append(X[3])



fig = plt.figure()
ax = fig.add_subplot(211)
ax.plot(x_t,z_t)
ax.scatter(x_0+x_vz,z_0+z_vz,label='Vel reached')





ax.axhline(y=H_CEIL,color='k',linestyle='--',label='h_ceiling')
ax.axhline(y=H_CEIL-d_vel,color='k',linestyle='--',alpha=0.5,label='d_vel',zorder=0)
ax.set_ylim(0,H_CEIL + 0.5)
ax.set_xlim(-10,10)

ax.set_xlabel('X_Pos [m]')
ax.set_ylabel('Z_Pos [m]')
ax.grid()
ax.legend(loc='lower right')

ax2 = fig.add_subplot(212)
ax2.plot(t_arr,Vz_t,label='V_z')
ax2.plot(t_arr,Vx_t,label='V_x')

ax2.set_xlabel('time [s]')
ax2.set_ylabel('Vel [m]')
ax2.grid()
ax2.legend(loc='upper left')


fig.tight_layout()
plt.show()




