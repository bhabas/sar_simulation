import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.05    # [m]

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 120
FPS = 120                # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3             # Focal Length [m]
O_x = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_y = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]


z_0 = 0.6     # Camera height [m]
vz = -1    # Camera velocity [m/s]

## PRE-ALLOCATE IMAGE ARRAY [pixels]
u_p = np.arange(0,WIDTH_PIXELS,1)
v_p = np.arange(0,HEIGHT_PIXELS,1)
U_p,V_p = np.meshgrid(u_p,v_p)



## PIXEL INTENSITIES/GRADIENTS
def I_continuous(u_p,z_0,t):
    ## CONVERT PIXEL INDEX TO METERS
    u = (u_p - O_x)*w + w/2 
    
    ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
    return I_0/2 * np.sin(2*np.pi*u*(z_0+vz*t)/(f*L)) + I_0/2


def I_pixel(u_p,z_0,t):

    ## CONVERT PIXEL INDEX TO METERS
    u = (u_p - O_x)*w + w/2 
    z = z_0 + vz*t
    ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
    return I_0/(2*w) * (f*L/(np.pi*z)*np.sin(2*np.pi*u*z/(f*L)) * np.sin(np.pi*w*z/(f*L)) + w)

def dI_dt_pixel(u_p,z_0,t):

    ## RETURN TIME GRADIENT VIA CENTRAL DIFFERENCE
    return (I_pixel(u_p,z_0,t+1/FPS) - I_pixel(u_p,z_0,t-1/FPS))/(2/FPS)

def dI_du_pixel(u_p,z_0,t):

    ## RETURN X-AXIS GRADIENT VIA CENTRAL DIFFERENCE
    return 1/(2*w)*(I_pixel(u_p+1,z_0,t) - I_pixel(u_p-1,z_0,t))



## IMAGE SENSOR PLOT
fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(I_pixel(U_p,z_0,0), interpolation='none', 
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

    U = (U_p - O_x)*w + w/2 
    V = (V_p - O_y)*w + w/2
    G = U[1:-1,1:-1]*Ix[1:-1,1:-1] + V[1:-1,1:-1]*Iy[1:-1,1:-1]

    Vz_est = -np.sum(G*It[1:-1,1:-1])/np.sum(G**2)

    # Vz_est = np.mean(-It[1:-1,1:-1]/G)
    return Vz_est


Vz_act_List = []
Vz_an_List = []
Vz_est_List = []
t_List = []


## ANIMATE PLOT
num_sec = 3
def animate_func(i):
    t = i/FPS # time [s]

    Cur_img = I_pixel(U_p,z_0,t)
    Prev_img = I_pixel(U_p,z_0,t-1/FPS)
    Vz_est = cam_alg(Cur_img,Prev_img)
    

    U = (U_p - O_x)*w + w/2 
    G = U[1:-1,1:-1]*dI_du_pixel(U_p,z_0,t)[1:-1,1:-1]

    Vz_an =-np.sum(G*dI_dt_pixel(U_p,z_0,t)[1:-1,1:-1])/np.sum(G**2)
    Vz_act = -vz/(z_0 + vz*t)

    print(f"Vz_act: {Vz_act:.3f} | Vz_an: {Vz_an:.3f} | Vz_est: {Vz_est:.3f}")

    Vz_act_List.append(1/Vz_act)
    Vz_an_List.append(1/Vz_an)
    Vz_est_List.append(1/Vz_est)
    t_List.append(t)
    ## UPDATE IMAGE
    # im.set_array(I_pixel(U_p,z_0,t))
    
anim = animation.FuncAnimation(fig, 
                               animate_func, 
                               frames = num_sec * FPS,
                               interval = 1000 / FPS, # in ms
                               repeat = False
                               )

plt.show()

## IMAGE SENSOR PLOT
fig2 = plt.figure(1)
ax = fig2.add_subplot(111)

# ax.plot(t_List,Vz_an_List,'--k',label="Tau_analytical")
ax.plot(t_List,Vz_est_List,'ro',label="Tau_algortihm")
ax.plot(t_List,Vz_act_List,label="Tau_actual")

ax.grid()
ax.legend()

fig2.tight_layout()

plt.show()



