import matplotlib.pyplot as plt
import numpy as np

## GOAL: FIND TRAJECTORY STARTING POSITIONS THAT ALLOW FOR DESIRED VELOCITIES TO BE REACHED


## FLIGHT CONDITIONS
V = 3.0     # [m/s]
phi = 40    # [deg]
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




def Vals(t_arr):
    x_t = np.zeros_like(t_arr)
    z_t = np.zeros_like(t_arr)

    Vx_t = np.zeros_like(t_arr)
    Vz_t = np.zeros_like(t_arr)

    OnceFlag = True
    t_impact = np.nan

    for ii,t in enumerate(t_arr):

        if t < t_x:
            x_t[ii] = 0.5*a_x*t**2 + x_0
            Vx_t[ii] = a_x*t

            z_t[ii] = z_0
            Vz_t[ii] = 0

        elif t_x <= t < t_x + t_z:
            x_t[ii] = V_x*t - V_x**2/(2*a_x) + x_0
            Vx_t[ii] = V_x

            z_t[ii] = 0.5*a_z*(t-t_x)**2 + z_0
            Vz_t[ii] = a_z*(t-t_x)

        elif t_x + t_z <= t:
            x_t[ii] = V_x*t - V_x**2/(2*a_x) + x_0
            Vx_t[ii] = V_x

            z_t[ii] = V_z*(t-t_x) - V_z**2/(2*a_z) + z_0
            Vz_t[ii] = V_z

        if z_t[ii] >= h_c and OnceFlag:
            OnceFlag = False
            t_impact = t_arr[ii]

    if len(t_arr) == 1:
        return x_t[0],Vx_t[0],z_t[0],Vz_t[0],t_impact
    else:
        return x_t,Vx_t,z_t,Vz_t,t_impact

x_t,Vx_t,z_t,Vz_t,t_impact = Vals(t_arr)
x_f,_,z_val,_,_ = Vals([t_x])
x_val,_,z_f,_,_ = Vals([t_x+t_z])

x_impact,_,z_impact,_,_ = Vals([t_impact])
# x_0 = -x_impact
# x_t,Vx_t,z_t,Vz_t,t_impact = Vals(t_arr)
# x_f,_,z_val,_,_ = Vals([t_x])
# x_val,_,z_f,_,_ = Vals([t_x+t_z])




fig = plt.figure()
ax = fig.add_subplot(211)
ax.plot(x_t,z_t)
ax.scatter(x_f,z_val,label='Vx reached')
ax.scatter(x_val,z_f,label='Vz reached')


ax.axhline(y=h_c,color='k',linestyle='--')
ax.set_ylim(0,h_c + 0.5)
ax.set_xlim(-5,2)

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



