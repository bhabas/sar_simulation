import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
import cv2 as cv


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.05    # [m]

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160
FPS = 100                # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3/2             # Focal Length [m]
# f_effective = f/2 # halve focal length if half the pixels
O_x = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_y = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]


z_0 = 1.3    # Camera height [m]
vz = -1    # Camera velocity [m/s]
vx = 0.0

## PRE-ALLOCATE IMAGE ARRAY [pixels]
u_p = np.arange(0,WIDTH_PIXELS,1)
v_p = np.arange(0,HEIGHT_PIXELS,1)
U_p,V_p = np.meshgrid(u_p,v_p)



## PIXEL INTENSITIES/GRADIENTS
def I_continuous(u_p,z_0,t):
    ## CONVERT PIXEL INDEX TO METERS
    u = (u_p - O_x)*w + w/2 
    d = z_0+vz*t

    ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
    return I_0/2 * np.sin(2*np.pi*(u*d/(f*L) + vx*t/L)) + I_0/2


def I_pixel(u_p,v_p,z_0,t):

    ## CONVERT PIXEL INDEX TO METERS
    u = (u_p - O_x)*w + w/2 
    v = (v_p - O_y)*w + w/2 

    z = z_0 + vz*t

    ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
    # I = I_0/2 * (f*L/(np.pi*z*w) * np.sin(np.pi*z*w/(f*L) * np.sin(2*np.pi*(z*u/(f*L) + vx*t/L))) + 1)

    I_x = I_0/2*(np.sin(2*np.pi*(z*u/(f*L) + vx*t/L) ) + 1)
    I_y = I_0/2*(np.sin(2*np.pi*(z*v/(f*L) + 0*t/L) ) + 1)
    I = (I_x+I_y)/2


    ## CLIP VALUES TO BE HIGH/LOW
    # I = np.round(I/255,0)*255

    return I



## IMAGE SENSOR PLOT
fig = plt.figure()
ax = fig.add_subplot(111)
im = ax.imshow(I_pixel(U_p,V_p,z_0,0), interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',
)
ax.set_title("Image Sensor Pattern (Pixels)")
ax.set_xlabel("u [pixels]")
ax.set_ylabel("v [pixels]")
fig.tight_layout()
# plt.show()

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

    Cur_img = cv.GaussianBlur(Cur_img,(5,5),0)
    Prev_img = cv.GaussianBlur(Prev_img,(5,5),0)

    for i in range(1,HEIGHT_PIXELS - 1): 
        for j in range(1,WIDTH_PIXELS - 1):
            Ix[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Kx)/(w)
            Iy[i,j] = -np.sum(Cur_img[i-1:i+2,j-1:j+2] * Ky)/(w)



    It = (Cur_img - Prev_img)/(1/FPS)

    U = (U_p - O_x)*w + w/2 
    V = (V_p - O_y)*w + w/2
    G = U*Ix + V*Iy

    X = np.array([
        [np.sum(Ix**2),np.sum(Ix*Iy),np.sum(G*Ix)],
        [np.sum(Ix*Iy),np.sum(Iy**2),np.sum(G*Iy)],
        [np.sum(G*Ix),np.sum(G*Iy),np.sum(G**2)]
    ])

    y = -np.array([[np.sum(Ix*It)],[np.sum(Iy*It)],[np.sum(G*It)]])

    b = np.linalg.pinv(X)@y

    
    return b


Tau_act_List = []
Tau_est_List = []
Vx_act_List = []
Vx_est_List = []
t_List = []


## ANIMATE PLOT
num_sec = 3
def animate_func(i):

    t_contact = z_0/np.abs(vz)
    t = i/FPS # time [s]
    if t>=t_contact:
        return

    Cur_img = I_pixel(U_p,V_p,z_0,t)
    Prev_img = I_pixel(U_p,V_p,z_0,t-1/FPS)
    b = cam_alg(Cur_img,Prev_img)
    d = (z_0 + vz*t)
    Tau_act = -d/vz
    Vx_act = -vx/d


    print(f"Tau_act: {Tau_act:.3f} | Tau_est: {1/b[2,0]:.3f}")
    print(f"Vx_act: {Vx_act:.3f} | Vx_est: {b[0,0]/f:.3f}")


    Tau_est_List.append(1/b[2,0])
    Tau_act_List.append(Tau_act)
    Vx_act_List.append(Vx_act)
    Vx_est_List.append(b[0,0]/f)


    t_List.append(t)

    ## UPDATE IMAGE
    # im.set_array(I_pixel(U_p,V_p,z_0,t))
    
anim = animation.FuncAnimation(fig, 
                               animate_func, 
                               frames = num_sec * FPS,
                               interval = 1000 / FPS, # in ms
                               repeat = False
                               )

plt.show()

fig2 = plt.figure(1)
ax = fig2.add_subplot(111)

ax.plot(t_List,Tau_est_List,'rx',label="Tau_algortihm")
ax.plot(t_List,Tau_act_List,label="Tau_actual")
ax.grid()
ax.legend()

fig2.tight_layout()



fig3 = plt.figure(2)
ax = fig3.add_subplot(111)

ax.plot(t_List,Vx_est_List,'rx',label="OFy_estimate")
ax.plot(t_List,Vx_act_List,label="OFy_actual")
ax.grid()
ax.legend()

fig3.tight_layout()

plt.show()



