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

w = 3.6e-6  # Pixel width [m]
f = 0.66e-3 # Focal Length [m]
O_y = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]
O_x = WIDTH_PIXELS/2    # Pixel X_offset [pixels]

FPS = 60    # Frame Rate [1/s]
z_0 = 0.6     # Camera distance [m]
vz = -0.2    # Camera velocity [m/s]
# Vx = vx/d   # Optical flow from x-vel [rad/s]
# print(f"Vx = {Vx:.3f} [rad/s]")


## IMAGE ARRAY [m]
u_p = np.arange(0,WIDTH_PIXELS,1)
v_p = np.arange(0,HEIGHT_PIXELS,1)
U_p,V_p = np.meshgrid(u_p,v_p)

# ## CONINTUOUS INTENSITY VALUES/GRADIENTS
def I(u,t):
    return I_0/2*(np.sin(2*np.pi*u*(z_0+vz*t)/(f*L))+1)





## PIXEL INTENSITIES/GRADIENTS
def I_pixel(u_p,t):

    ## CONVERT PIXEL INDEX TO METERS
    u = (u_p - O_x)*w + w/2 

    
    ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
    return I_0/2*(np.sin(2*np.pi*u*(z_0+vz*t)/(f*L))+1)


## IMAGE SENSOR PLOT
fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(I_pixel(U_p,0), interpolation='none', 
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
    t = i/FPS # time [s]

    im.set_array(I_pixel(U_p,t))
    return [im]

anim = animation.FuncAnimation(fig, 
                               animate_func, 
                               frames = num_sec * FPS,
                               interval = 1000 / FPS, # in ms
                               )

plt.show()







