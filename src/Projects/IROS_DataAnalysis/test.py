import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.stats import truncnorm  
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")


lower1 = 0
lower2 = 8

## X-PLOTS
mu1 = 1.0
sig1 = 0.75

a, b = (lower1 - mu1) / sig1, (lower2 - mu1) / sig1
x1 = np.linspace(-0.01,5,100)
y1 = np.zeros_like(x1)
z1 = truncnorm.pdf(x1, a, b, loc = mu1, scale = sig1)
ax.plot(x1,y1,z1,"b")


mu2 = 3.0
sig2 = 0.68

a, b = (lower1 - mu2) / sig2, (lower2 - mu2) / sig2
x2 = np.linspace(-0.010,5,100)
y2 = np.zeros_like(x2)
z2 = truncnorm.pdf(x2, a, b, loc = mu2, scale = sig2)
ax.plot(x2,y2,z2,"b--")



## Y-PLOTS
mu3 = 2
sig3 = 0.6
a, b = (lower1 - mu3) / sig3, (lower2 - mu3) / sig3

y3 = np.linspace(0,5,100)
x3 = np.zeros_like(y3)-2
z3 = truncnorm.pdf(y3, a, b, loc = mu3, scale = sig3)
ax.plot(x3,y3,z3,"r")


mu4 = 4
sig4 = 0.6
a, b = (lower1 - sig4) / sig3, (lower2 - mu4) / sig4

y4 = np.linspace(0,5,100)
x4 = np.zeros_like(y3)-2
z4 = truncnorm.pdf(y3, a, b, loc = mu4, scale = sig4)
ax.plot(x4,y4,z4,"r--")






ax.set_xlim(-2,5)
ax.set_ylim(5,0)
ax.set_zlim(0,0.7)


fig.tight_layout()
plt.show()