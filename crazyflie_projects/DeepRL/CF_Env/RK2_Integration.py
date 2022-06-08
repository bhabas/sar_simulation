import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


dt = 0.05



def simulate(dt,c):
    z = 0.1
    vz = 5.0
    g = 9.81
    t = 0

    m = 1.0

    z_list = []
    t_list = []

    z_list.append(z)
    t_list.append(t)

    while z > 0:
        
        
        az = -g - np.sign(vz)*c/m*np.abs(vz)**2
        z_mid = z + vz*(0.5*dt)
        vz_mid = vz + az*(0.5*dt)
        az_mid = -g - np.sign(vz_mid)*c/m*np.abs(vz_mid)**2

        z += vz_mid*dt
        vz += az_mid*dt

        t += dt

        z_list.append(z)
        t_list.append(t)

        
    return t_list,z_list

t1,z1 = simulate(dt=0.001,c=2.0)
t2,z2 = simulate(dt=0.001,c=0.1)
t3,z3 = simulate(dt=0.001,c=0.01)


fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(t1,z1)
ax.plot(t2,z2)
ax.plot(t3,z3)
ax.set_ylim(0,1)
ax.set_ylabel("Height [m]")
ax.set_xlabel("Time [s]")
ax.grid()
plt.show()


