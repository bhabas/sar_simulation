import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.patches import Rectangle


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.2     # [m]

## CAMERA PARAMETERS
f = 0.66e-3         # Focal Length [m]
d = 0.6             # Camera distance [m]


u = np.arange(-80,80,1)*3.6e-6
v = np.arange(-60,60,1)*3.6e-6
U,V = np.meshgrid(u,v)

def Intensity(u):
    return I_0/2*(np.sin(2*np.pi/L*(u*d/f))+1)

fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(Intensity(U), interpolation='bilinear', 
                vmin=0, vmax=255, cmap=cm.Greys,
                origin='upper',
                extent=[np.min(u),np.max(u),np.min(v),np.max(v)]
)
ax.set_title("Image Sensor Pattern")
ax.set_xlabel("u [m]")
ax.set_ylabel("v [m]")
fig.tight_layout()
plt.show()
