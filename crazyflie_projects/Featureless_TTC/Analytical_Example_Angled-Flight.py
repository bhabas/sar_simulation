import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
# import cv2 as cv


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.45    # [m]

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160
FPS = 60               # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3/2          # Focal Length [m]
# f_effective = f/2 # halve focal length if half the pixels
O_up = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_vp = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]


d_0 = 1.7   # Initial Camera height [m]
vz = 2.5   # Camera velocity [m/s]
vx = 0.75
vy = 0.0

## PRE-ALLOCATE IMAGE ARRAY [pixels]
u_p = np.arange(0,WIDTH_PIXELS,1)
v_p = np.arange(0,HEIGHT_PIXELS,1)
U_p,V_p = np.meshgrid(u_p,v_p)



## PIXEL INTENSITIES/GRADIENTS
def I_continuous(u_p,z_0,t):
    ## CONVERT PIXEL INDEX TO METERS
    u = (u_p - O_up)*w + w/2 
    d = z_0+vz*t

    ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
    return I_0/2 * np.sin(2*np.pi*(u*d/(f*L) + vx*t/L)) + I_0/2


def I_pixel(u_p,v_p,d_0,t):

    ## CONVERT PIXEL INDEX TO METERS
    u = (u_p - O_up)*w + w/2 
    v = (v_p - O_vp)*w + w/2 

    d = d_0 - vz*t

    ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
    I_x = I_0/2*(np.sin(2*np.pi*(d*u/(f*L) + vx*t/L) ) + 1)
    I_y = I_0/2*(np.sin(2*np.pi*(d*v/(f*L) + vy*t/L) ) + 1)
    I = I_x


    ## CLIP VALUES TO BE HIGH/LOW
    # I = np.round(I/255,0)*255

    return I



## IMAGE SENSOR PLOT
fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(I_pixel(U_p,V_p,d_0,0), interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',
)
ax.set_title("Image Sensor Pattern (Pixels)")
ax.set_xlabel("u [pixels]")
ax.set_ylabel("v [pixels]")
fig.tight_layout()
# plt.show()

def cam_alg(Cur_img,Prev_img):

    Kx = 1/8 * np.array([ # NORMALIZED SOBEL KERNAL (U-DIRECTION)
        [-1,0,1],
        [-2,0,2],
        [-1,0,1]
    ]) 
    Ky = 1/8 *np.array([ # NORMALIZED SOBEL KERNAL (V--DIRECTION)
        [-1,-2,-1],
        [ 0, 0, 0],
        [ 1, 2, 1]
    ])

    Iu = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
    Iv = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))

    U_grid = (U_p - O_up)*w + w/2 
    V_grid = (V_p - O_vp)*w + w/2

    ## FIND IMAGE GRADIENTS
    for i in range(1,HEIGHT_PIXELS - 1): 
        for j in range(1,WIDTH_PIXELS - 1):
            Iu[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Kx)/w
            Iv[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Ky)/w

    It = (Cur_img - Prev_img)/(1/FPS) # Time Gradient
    G = U_grid*Iu + V_grid*Iv # Radial Gradient

    ## SOLVE LEAST SQUARES PROBLEM
    X = np.array([
        [f*np.sum(Iu**2), f*np.sum(Iu*Iv), np.sum(G*Iu)],
        [f*np.sum(Iu*Iv), f*np.sum(Iv**2), np.sum(G*Iv)],
        [f*np.sum(G*Iu),  f*np.sum(G*Iv),  np.sum(G**2)]
    ])

    y = np.array([
                [-np.sum(Iu*It)],
                [-np.sum(Iv*It)],
                [-np.sum(G*It)]
            ])

    ## SOLVE b VIA PSEUDO-INVERSE
    b = np.linalg.pinv(X)@y
    b = b.flatten()

    
    return b


Tau_act_List = []
Tau_est_List = []
OFy_act_List = []
OFy_ext_List = []
OFx_act_List = []
OFx_ext_List = []
t_List = []


## ANIMATE PLOT
num_sec = 3
def animate_func(i):


    ## UPDATE CAMERA POSITION
    t = i/FPS   # time [s]

    d = (d_0 - vz*t)    # Distance to Ceiling [m]
    Tau_act = d/vz
    OFy_act = -vx/d
    OFx_act = -vy/d

    ## STOP ANIMATION IF PAST CEILING SURFACE
    if Tau_act <= 0:
        return
    

    ## CALCULATE OPTICAL FLOW ESTIMATES
    Cur_img = I_pixel(U_p,V_p,d_0,t)
    Prev_img = I_pixel(U_p,V_p,d_0,t-1/FPS)
    b = cam_alg(Cur_img,Prev_img)
    
    print(f"Tau_act: {Tau_act:.3f} | Tau_est: {1/b[2]:.3f}")
    print(f"OFy_act: {OFy_act:.3f} | OFy_est: {b[0]:.3f}")
    print(f"OFx_act: {OFx_act:.3f} | OFx_est: {b[1]:.3f}\n")


    ## APPEN OPTICAL FLOW ESTIMATES TO LIST FOR PLOTTING
    Tau_est_List.append(1/b[2])
    Tau_act_List.append(Tau_act)
    OFy_act_List.append(OFy_act)
    OFy_ext_List.append(b[0])
    OFx_act_List.append(OFx_act)
    OFx_ext_List.append(b[1])

    t_List.append(t)

    ## UPDATE IMAGE
    im.set_array(I_pixel(U_p,V_p,d_0,t))
    
anim = animation.FuncAnimation(fig, 
                               animate_func, 
                               frames = num_sec * FPS,
                               interval = 1000 / FPS, # in ms
                               repeat = False
                               )

plt.show()

## TAU PLOT
fig2 = plt.figure(1)
ax = fig2.add_subplot(111)

ax.plot(t_List,Tau_est_List,'rx',label="Tau_estimate")
ax.plot(t_List,Tau_act_List,label="Tau_actual")
ax.set_title('Tau Estimation')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Tau [s]')
ax.grid()
ax.legend()

fig2.tight_layout()


## OFy PLOT
fig3 = plt.figure(2)
ax = fig3.add_subplot(111)

ax.plot(t_List,OFy_ext_List,'rx',label="OFy_estimate")
ax.plot(t_List,OFy_act_List,label="OFy_actual")
ax.set_title('OFy Estimation')
ax.set_xlabel('Time [s]')
ax.set_ylabel('OFy [rad/s]')
ax.grid()
ax.legend()

fig3.tight_layout()

## OFx PLOT
fig4 = plt.figure(3)
ax = fig4.add_subplot(111)

ax.plot(t_List,OFx_ext_List,'rx',label="OFx_estimate")
ax.plot(t_List,OFx_act_List,label="OFx_actual")
ax.set_title('OFx Estimation')
ax.set_xlabel('Time [s]')
ax.set_ylabel('OFx [rad/s]')
ax.grid()
ax.legend()

fig4.tight_layout()

plt.show()



