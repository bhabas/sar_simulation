import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.patches import Rectangle


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.2    # [m]

## CAMERA PARAMETERS
f = 0.66e-3 # Focal Length [m]
d = 0.6     # Camera distance [m]
w = 3.6e-6  # Pixel width [m]
O_x = 80    # Pixel X_offset [pixels]
O_y = 60    # Pixel Y_offset [pixels]
FPS = 60    # Frame Rate [1/s]
vx = 0.1    # Camera velocity [m/s]

Vx = vx/d   # Optical flow from x-vel [rad/s]
print(f"Vx = {Vx:.3f} [rad/s]")


## GRADIENT ERRORS
gamma_d = np.pi*d*w/(f*L)
gamma_x = 2*np.pi*vx/(FPS*L)
print(f"Gamma_d = {gamma_d:.3f}")
print(f"Gamma_x = {gamma_x:.3f}")

## IMAGE ARRAY [m]
u_p = np.arange(0,160,1)
v_p = np.arange(0,120,1)
U_p,V_p = np.meshgrid(u_p,v_p)

def Intensity(u_p,i):
    u = (u_p - O_x)*w + w/2
    return I_0/2 * f*L/(np.pi*d*w) * np.sin(np.pi*d*w/(f*L)) * np.sin(2*np.pi*(u*d/(f*L) + vx*i/L)) + I_0/2

## CREATE INITIAL PLOT
fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(Intensity(U_p,0), interpolation='none', 
                vmin=0, vmax=255, cmap=cm.Greys,
                origin='upper',
)
ax.set_title("Image Sensor Pattern (Pixels)")
ax.set_xlabel("u [pixels]")
ax.set_ylabel("v [pixels]")
fig.tight_layout()

## ANIMATE PLOT
num_sec = 5
def animate_func(i):
    if i % FPS == 0:
        print( '.', end ='' )

    im.set_array(Intensity(U_p,i/FPS))
    return [im]

anim = animation.FuncAnimation(fig, 
                               animate_func, 
                               frames = num_sec * FPS,
                               interval = 1000 / FPS, # in ms
                               )

plt.show()
