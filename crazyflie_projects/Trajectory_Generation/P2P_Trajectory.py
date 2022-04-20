import matplotlib.pyplot as plt
import numpy as np


def Theta(t,T,theta_0,theta_f):
    
    return theta_0 + (3*t**2/T**2 - 2*t**3/T**3)*(theta_f - theta_0)

def dTheta(t,T,theta_0,theta_f):

    return (6*t/T**2 - 6*t**2/T**3)*(theta_f - theta_0)

def ddTheta(t,T,theta_0,theta_f):

    return (6/T**2 - 12*t/T**3)*(theta_f - theta_0)


a_max = 1.0 # [m/s^2]
x_0 = 0.4
x_f = 1.4

T = np.sqrt(6/a_max*np.abs(x_f - x_0))
t = np.linspace(0,T,50)


fig = plt.figure()
ax = fig.add_subplot(311)
ax.plot(t,Theta(t,T,x_0,x_f))

ax.set_xlim(0,5)
ax.set_ylim(0,5)
ax.set_xlabel('Time [s]')
ax.set_ylabel('Position [m]')
ax.grid()

ax2 = fig.add_subplot(312)
ax2.plot(t,dTheta(t,T,x_0,x_f))

ax2.set_xlim(0,5)
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Velocity [m/s]')
ax2.grid()


ax3 = fig.add_subplot(313)
ax3.plot(t,ddTheta(t,T,x_0,x_f))

ax3.set_xlim(0,5)
ax3.set_xlabel('Time [s]')
ax3.set_ylabel('Acceleration [m/s^2]')
ax3.grid()


fig.tight_layout()
plt.show()






# fig = plt.figure()
# ax = fig.add_subplot(211)
# ax.plot(x_t,z_t)
# ax.scatter(x_f,z_val,label='Vx reached')
# ax.scatter(x_val,z_f,label='Vz reached')


# ax.axhline(y=h_c,color='k',linestyle='--')
# ax.set_ylim(0,h_c + 0.5)
# ax.set_xlim(-5,2)

# ax.set_xlabel('X_Pos [m]')
# ax.set_ylabel('Z_Pos [m]')
# ax.grid()
# ax.legend(loc='upper left')

# ax2 = fig.add_subplot(212)
# ax2.plot(t_arr,Vz_t,label='V_z')
# ax2.plot(t_arr,Vx_t,label='V_x')

# ax2.set_xlabel('time [s]')
# ax2.set_ylabel('Vel [m]')
# ax2.grid()
# ax2.legend(loc='upper left')


# fig.tight_layout()
# plt.show()



