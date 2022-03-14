import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.025    # [m]

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 120
FPS = 60                # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3             # Focal Length [m]
O_x = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_y = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]


d = 0.6     # Camera distance [m]
vx = 0.2    # Camera velocity [m/s]

## PRE-ALLOCATE IMAGE ARRAY [pixels]
u_p = np.arange(0,WIDTH_PIXELS,1)
v_p = np.arange(0,HEIGHT_PIXELS,1)
U_p,V_p = np.meshgrid(u_p,v_p)



## PIXEL INTENSITIES/GRADIENTS
def I_pixel(u_p,t):

    ## CONVERT PIXEL INDEX TO METERS
    u = (u_p - O_x)*w + w/2 
    
    ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
    return I_0/2 * f*L/(np.pi*d*w) * np.sin(np.pi*d*w/(f*L)) * np.sin(2*np.pi*(u*d/(f*L) + vx*t/L)) + I_0/2 

def dI_dt_pixel(u_p,t):

    ## RETURN TIME GRADIENT VIA CENTRAL DIFFERENCE
    return (I_pixel(u_p,t+1/FPS) - I_pixel(u_p,t-1/FPS))/(2/FPS)

def dI_du_pixel(u_p,t):

    ## RETURN X-AXIS GRADIENT VIA CENTRAL DIFFERENCE
    return 1/(2*w)*(I_pixel(u_p+1,t) - I_pixel(u_p-1,t))



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

def cam_alg(Cur_img,Prev_img):

    Kx = 1/8 * np.array([ # SOBEL KERNEL X
        [-1,0,1],
        [-2,0,2],
        [-1,0,1]
    ]) 
    Ky = 1/8 *np.array([ # SOBEL KERNEL Y
        [ 1, 2, 1],
        [ 0, 0, 0],
        [-1,-2,-1]])

    Ix = np.zeros_like(Cur_img)
    Iy = np.zeros_like(Cur_img)

    for i in range(1,HEIGHT_PIXELS - 1): 
        for j in range(1,WIDTH_PIXELS - 1):
            Ix[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Kx)/(w)
            Iy[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Ky)/(w)

    It = (Cur_img - Prev_img)/(1/FPS)

    Vx = np.mean(1/f*It[1:-1,1:-1]/Ix[1:-1,1:-1])    

    return Vx


Vx_an_List = []
Vx_est_List = []

## ANIMATE PLOT
num_sec = 1
def animate_func(i):
    t = i/FPS # time [s]

    Cur_img = I_pixel(U_p,t)
    Prev_img = I_pixel(U_p,t-1/FPS)
    Vx_est = cam_alg(Cur_img,Prev_img)
    Vx_an = np.mean(1/f*dI_dt_pixel(U_p,t)[1:-1,1:-1]/dI_du_pixel(U_p,t)[1:-1,1:-1])
    print(f"Vx: {Vx_an:.3f} | Vx_est: {Vx_est:.3f}")

    Vx_an_List.append(Vx_an)
    Vx_est_List.append(Vx_est)
    
    ## UPDATE IMAGE
    im.set_array(I_pixel(U_p,t))
    
anim = animation.FuncAnimation(fig, 
                               animate_func, 
                               frames = num_sec * FPS,
                               interval = 1000 / FPS, # in ms
                               )

plt.show()


pass
