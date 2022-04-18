import matplotlib.pyplot as plt
import numpy as np

## GOAL: FIND TRAJECTORY STARTING POSITIONS THAT ALLOW FOR DESIRED VELOCITIES TO BE REACHED


## FLIGHT CONDITIONS
V = 3.0     # [m/s]
phi = 60    # [deg]
h_c = 2.1

## VELOCITY REACHED LOCATION
D_f = 0.5
X_f = 0.0

## SYSTEM ACCELERATIONS
a_z = 3.1   # [m/s^2]
a_x = 1.0   # [m/s^2]


phi_rad = np.radians(phi)
V_z = V*np.sin(phi_rad)
V_x = V*np.cos(phi_rad)

t_z = V_z/a_z   # Time Vz reached
t_x = V_x/a_x   # Time Vx reached
t_arr = np.linspace(0,5,100)

z_0 = 0.4
x_0 = 0


def Vals(t_arr,t_sf,s_0,V_s,a_s):

    s_t = np.zeros_like(t_arr)
    Vs_t = np.zeros_like(t_arr)
   
    for ii,t in enumerate(t_arr):

        if t < t_sf:
            s_t[ii] = 0.5*a_s*t**2 + s_0
            Vs_t[ii] = a_s*t

        elif t_sf <= t:
            s_t[ii] = V_s*t - V_s**2/(2.0*a_s) + s_0
            Vs_t[ii] = V_s

    if len(t_arr) == 1:
        return s_t[0],Vs_t[0]
    
    else:
        return s_t,Vs_t


x_t,Vx_t = Vals(t_arr,t_x,x_0,V_x,a_x)
z_t,Vz_t = Vals(t_arr,t_z,z_0,V_z,a_z)

## CALC POSITION WHEN TIME REACHED
x_f,Vx_f = Vals([t_x],t_x,x_0,V_x,a_x)
z_f,Vz_f = Vals([t_z],t_z,z_0,V_z,a_z)



fig = plt.figure()
ax = fig.add_subplot(211)
ax.plot(x_t,z_t)
ax.scatter(Vals([t_z],t_x,x_0,V_x,a_x)[0],z_f,label='Vz reached')
ax.scatter(x_f,Vals([t_x],t_z,z_0,V_z,a_z)[0],label='Vx reached')


ax.axhline(y=h_c,color='k',linestyle='--')
ax.set_ylim(0,h_c + 0.5)
ax.set_xlim(-1,1)

ax.set_xlabel('X_Pos [m]')
ax.set_ylabel('Z_Pos [m]')
ax.grid()
ax.legend()

ax2 = fig.add_subplot(212)
ax2.plot(t_arr,Vz_t,label='V_z')
ax2.plot(t_arr,Vx_t,label='V_x')
ax2.grid()

ax2.set_xlabel('time [s]')
ax2.set_ylabel('Vel [m]')
ax2.legend()


fig.tight_layout()
plt.show()



