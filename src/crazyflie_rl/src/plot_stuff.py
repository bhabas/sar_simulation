import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

theta = np.linspace(10,-200,50)
r_theta = np.zeros_like(theta)

for ii,theta_ii in enumerate(theta):
    
    if -170 < theta_ii <= 0:
        r_theta[ii] = 5*(-1/170*theta_ii)
    elif -190 <= theta_ii <= -170:
        r_theta[ii] = 5
    else:
        r_theta[ii] = 0
    

# fig = plt.figure(0)
# ax = fig.add_subplot(111)
# ax.set_ylabel('r_theta')
# ax.set_xlabel('theta [deg]')
# ax.set_xlim([-200,10])
# ax.set_xticks([-200,-180,-135,-90,-45,0,15])
# ax.grid()

# plt.plot(theta,r_theta)


lc = np.repeat([0,1,2,3,4],10)
r_lc = np.exp(lc/2.5)

fig = plt.figure(1)
ax = fig.add_subplot(111)
ax.set_xlabel('Leg Contacts')
ax.set_ylabel('r_lc')
ax.set_title('r_lc vs #Leg Contacts')
ax.grid()
plt.scatter(lc,r_lc)
plt.show()



W_My = np.linspace(0,12.5,50)
r_W = 10*np.exp(-W_My/5)

# fig = plt.figure(1)
# ax = fig.add_subplot(111)
# ax.set_xlabel('W_My [N*mm]')
# ax.set_ylabel('r_W')
# ax.grid()

# plt.plot(W_My,r_W)
# plt.show()


r_l = r_lc + r_theta
R_W,R_L = np.meshgrid(r_W,r_l)
R = R_W*R_L


fig = plt.figure(2)
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(R_W,R_L,R,cmap=cm.jet)
ax.set_xlabel('r_W')
ax.set_ylabel('r_L')
ax.set_zlabel('R')
ax.set_zlim([0,110])
ax.grid()
plt.show()
