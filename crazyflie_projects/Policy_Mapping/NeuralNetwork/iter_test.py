import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


x = np.linspace(-5,5,11)
y = np.linspace(-5,5,11)
XX,YY = np.meshgrid(x,y)

ZZ = XX*YY

for ii,val in enumerate(ZZ.flatten()):
    print(f"({XX.flatten()[ii]},{YY.flatten()[ii]}) -> {ZZ.flatten()[ii]}")
    print(f"({XX.flatten()[ii]},{YY.flatten()[ii]}) -> {val}")

    # You'll need to play around with array ordering to start in upper left corner
    # and make sure you have the correct values (Check edge-cases like asymmetric patterns to verify)
    # UL Pixel->(0,0) so start algorithm in UL Corner->(-5,5)


## PLOT SETUP
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(XX.flatten(),YY.flatten(),c=ZZ.flatten())
# ax.scatter(XX.flatten()[0:3],YY.flatten()[0:3],c=ZZ.flatten()[0:3])


ax.set_xlim(-6,6)
ax.set_ylim(-6,6)

ax.set_xlabel("x")
ax.set_ylabel("y")

ax.grid()
ax.set_axisbelow(True)

plt.show()
