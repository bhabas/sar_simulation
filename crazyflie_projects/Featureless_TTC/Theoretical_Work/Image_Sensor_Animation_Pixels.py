import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.patches import Rectangle


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.2    # [m]

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 120
w = 3.6e-6  # Pixel width [m]

f = 0.66e-3 # Focal Length [m]
O_x = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_y = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]

FPS = 60    # Frame Rate [1/s]
d = 0.6     # Camera distance [m]
vx = 0.2    # Camera velocity [m/s]
Vx = vx/d   # Optical flow from x-vel [rad/s]
print(f"Vx = {Vx:.3f} [rad/s]")


## GRADIENT ERRORS
gamma_d = np.pi*d*w/(f*L)
gamma_x = 2*np.pi*vx/(FPS*L)
print(f"Gamma_d = {gamma_d:.3f}")
print(f"Gamma_x = {gamma_x:.3f}")
print("\n\n")

## IMAGE ARRAY [m]
u_p = np.arange(0,WIDTH_PIXELS,1)
v_p = np.arange(0,HEIGHT_PIXELS,1)
U_p,V_p = np.meshgrid(u_p,v_p)

## CONINTUOUS INTENSITY VALUES/GRADIENTS
def Intensity(u,t):
    return I_0/2 * np.sin(2*np.pi/L * (u*d/f + vx*t)) + I_0/2

def T_grad(u,t):
    return 2*np.pi*vx/L * I_0/2 * np.cos(2*np.pi/L * (u*d/f + vx*t))

def U_grad(u,t):
    return 2*np.pi*d/(f*L) * I_0/2 * np.cos(2*np.pi/L * (u*d/f + vx*t))



## PIXEL INTENSITIES/GRADIENTS
def Intensity_pixel(u_p,t):

    ## CONVERT PIXEL INDEX TO METERS
    u = (u_p - O_x)*w + w/2 
    
    ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
    return I_0/2 * f*L/(np.pi*d*w) * np.sin(np.pi*d*w/(f*L)) * np.sin(2*np.pi*(u*d/(f*L) + vx*t/L)) + I_0/2 

def T_grad_pixel(u_p,t):

    ## RETURN TIME GRADIENT VIA CENTRAL DIFFERENCE
    return FPS/2*(Intensity_pixel(u_p,t+1/FPS) - Intensity_pixel(u_p,t-1/FPS))

def U_grad_pixel(u_p,t):

    ## RETURN X-AXIS GRADIENT VIA CENTRAL DIFFERENCE
    return 1/(2*w)*(Intensity_pixel(u_p+1,t) - Intensity_pixel(u_p-1,t))


# u = -286.2e-6   # [m]
# u_p = 0         # [pixels]


# print(f"Intensity: {Intensity(u,0):.3f}")
# print(f"T_grad: {T_grad(u,0):.3f}")
# print(f"U_grad: {U_grad(u,0):.3E}")
# print(f"Vx: {T_grad(u,0)/U_grad(u,0)*1/f:.3f}")
# print(f"================")


# print(f"Pixel Intensity: {Intensity_pixel(u_p,0):.3f}")
# print(f"T_grad_p: {T_grad_pixel(u_p,0):.3f}")
# print(f"U_grad_P: {U_grad_pixel(u_p,0):.3E}")
# print(f"Vx_P: {T_grad_pixel(u,0)/U_grad_pixel(u,0)*1/f:.3f}")







## IMAGE SENSOR PLOT
fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(Intensity_pixel(U_p,0), interpolation='none', 
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

    Kx = 1/4 * np.array([ # SOBEL KERNEL X
        [-1,0,1],
        [-2,0,2],
        [-1,0,1]
    ]) 
    Ky = np.array([ # SOBEL KERNEL Y
        [ 1, 2, 1],
        [ 0, 0, 0],
        [-1,-2,-1]])
    
    Cur_img = Intensity_pixel(U_p,t)
    Ix = np.zeros_like(Cur_img)

    for i in range(1,Cur_img.shape[0] - 1): #Calculate Radial gradient G
        for j in range(1,Cur_img.shape[1] - 1):
            Ix[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Kx)/(2*w)
            
            # Iy[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Ky)

    print(i/FPS)
    # im.set_array(Intensity_pixel(U_p,t))
    return [Ix]

anim = animation.FuncAnimation(fig, 
                               animate_func, 
                               frames = num_sec * FPS,
                               interval = 1000 / FPS, # in ms
                               )

plt.show()







