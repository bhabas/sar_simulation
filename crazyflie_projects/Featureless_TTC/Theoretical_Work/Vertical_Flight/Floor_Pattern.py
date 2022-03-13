import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.patches import Rectangle


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.2     # [m]

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 120
FPS = 60    # Frame Rate [1/s]
w = 3.6e-6  # Pixel width [m]

f = 0.66e-3 # Focal Length [m]
O_x = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_y = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]

d_0 = 0.6     # Camera distance [m]
vz = 0.2    # Camera velocity [m/s]

## DEFINE INTENSITY OF FLOOR GRID
x = np.linspace(-0.5, 0.5, 1000)    # [m]
y = np.linspace(-0.5, 0.5, 1000)    # [m]
X, Y = np.meshgrid(x, y)


def Intensity(X):
    return I_0/2*(np.sin(2*np.pi*X/L)+1)


## PLOT IMAGE OF FLOOR
fig, ax = plt.subplots()
im = ax.imshow(Intensity(X), interpolation='bilinear', 
                vmin=0, vmax=255, cmap=cm.Greys,
                origin='upper',
                extent=[-0.5,0.5,-0.5,0.5]
)


ax.set_title("Floor Pattern")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
fig.tight_layout()

plt.show()
