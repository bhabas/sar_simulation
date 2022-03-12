import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.patches import Rectangle


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.05    # [m]

## CAMERA PARAMETERS
f = 0.66e-3 # Focal Length [m]
d = 0.6     # Camera distance [m]
w = 3.6e-6  # Pixel width [m]
FPS = 60    # Frame Rate [1/s]
vx = 1.0    # Camera velocity [m/s]

Vx = vx/d   # Optical flow from x-vel [rad/s]
print(f"Vx = {Vx:.3f} [rad/s]")


## GRADIENT ERRORS
gamma_d = np.pi*d*w/(f*L)
gamma_x = 2*np.pi*vx/(FPS*L)
print(f"Gamma_d = {gamma_d:.3f}")
print(f"Gamma_x = {gamma_x:.3f}")


u = np.arange(-80,80,1)*3.6e-6
v = np.arange(-60,60,1)*3.6e-6
U,V = np.meshgrid(u,v)

def Intensity(u,i):
    return I_0/2*(np.sin(2*np.pi/L*(u*d/f+vx*i))+1)

## CREATE INITIAL PLOT
fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(Intensity(U,0), interpolation='bilinear', 
                vmin=0, vmax=255, cmap=cm.Greys,
                origin='upper',
                extent=[np.min(u),np.max(u),np.min(v),np.max(v)]
)
ax.set_title("Image Sensor Pattern")
ax.set_xlabel("u [m]")
ax.set_ylabel("v [m]")
fig.tight_layout()


fps = 60
num_sec = 5
def animate_func(i):
    if i % fps == 0:
        print( '.', end ='' )

    im.set_array(Intensity(U,i/fps))
    return [im]

anim = animation.FuncAnimation(fig, 
                               animate_func, 
                               frames = num_sec * fps,
                               interval = 1000 / fps, # in ms
                               )

plt.show()
