import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.patches import Rectangle


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.2     # [m]

## CAMERA PARAMETERS
u_min = 3.6e-6*80   # Image Sensor min [m]
v_min = 3.6e-6*60   # Image Sensor min [m]
f = 0.66e-3         # Focal Length [m]
d = 0.6             # Camera distance [m]

X_min_c = u_min*d/f
Y_min_c = v_min*d/f

x = np.linspace(-0.5, 0.5, 1000)    # [m]
y = np.linspace(-0.5, 0.5, 1000)    # [m]
X, Y = np.meshgrid(x, y)

def Intensity(X):
    return I_0/2*(np.sin(2*np.pi*X/L)+1)

fig, ax = plt.subplots()
im = ax.imshow(Intensity(X), interpolation='bilinear', 
                vmin=0, vmax=255, cmap=cm.Greys,
                origin='upper',
                extent=[-0.5,0.5,-0.5,0.5]
)

ax.add_patch(Rectangle((-X_min_c,-Y_min_c),X_min_c*2,Y_min_c*2,lw=1.5,fill=False,color="tab:blue"))

ax.set_title("Floor Pattern")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
fig.tight_layout()

plt.show()
