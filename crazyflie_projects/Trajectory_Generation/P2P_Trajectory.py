import matplotlib.pyplot as plt
import numpy as np

def Theta(s,theta_0,theta_f):

    theta = theta_0 + s*(theta_f - theta_0)

    return theta

def s(t,T):
    a_0 = 0
    a_1 = 0
    a_2 = 3/T**2
    a_3 = -2/T**3

    return a_0 + a_1*t + a_2*t**2 + a_3*t**3

t = np.linspace(0,2.5,50)
s_val = s(t,t[-1])

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(t,Theta(s_val,0,3))

ax.set_xlim(0,5)
ax.set_ylim(0,5)
ax.set_xlabel('Time [s]')
ax.set_ylabel('Position [m]')
ax.grid()

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



