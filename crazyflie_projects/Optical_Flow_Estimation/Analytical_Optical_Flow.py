import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(1,BASE_PATH)


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3/2          # Focal Length [m]
O_up = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_vp = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]

class Optical_Flow():

    def __init__(self,L,FPS):

        self.L = L
        self.FPS = FPS

    def Generate_Image(self,D_0=1,Vx=0,Vz=0,t=0):

        U_p,V_p = np.meshgrid(np.arange(0,WIDTH_PIXELS,1),np.arange(0,HEIGHT_PIXELS,1))

        ## CONVERT PIXEL INDEX TO METERS
        u = -(U_p - O_up)*w + w/2 
        v =  (U_p - O_up)*w + w/2

        D = D_0-Vz*t

        ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
        return I_0/2 * np.sin(2/L*np.pi * (u*D/f + Vx*t)) + I_0/2

    def Generate_Pattern(self,width=2,height=2,pixel_density=100,save_img=False):

        x = np.linspace(-width,width,pixel_density*width)
        y = np.linspace(-height,height,pixel_density*height)
        X,Y = np.meshgrid(x,y)

        I = I_0/2*(np.sin(2*np.pi/self.L*X) + 1)

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.add_patch(Rectangle((-1,-1),2,2,lw=2,fill=False,color="tab:blue"))
        ax.imshow(I, interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',extent=[x.min(),x.max(),y.min(),y.max()])

        plt.show()

        if save_img == True:
            plt.imsave(
                f'{BASE_PATH}/crazyflie_projects/Optical_Flow_Estimation/Surface_Patterns/Stripe.png', 
                I, 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',
            )



    def Plot_Image(self,image):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        im = ax.imshow(image, interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',extent=[-4,4,-4,4])

        plt.show()

    def OF_Calc_Opt_Sep(self,cur_img,prev_img,delta_t):

        """Calculate optical flow values with seperable convolution and integer optimizations.
        Derivation in (Research_Notes_Book_2.pdf)

        Args:
            cur_img (np.array): Array of current image
            prev_img (np.array): Array of previous image
            delta_t (float): Time between images

        Returns:
            np.array: Array of (Theta_x_est,Theta_y_est,Theta_z_est)
        """        

        ## SEPERATED SOBEL KERNAL (U--DIRECTION)
        Ku_1 = np.array([[-1,0,1]]).reshape(3,1)
        Ku_2 = np.array([[ 1,2,1]]).reshape(1,3)


        ## SEPERATED SOBEL KERNAL (V--DIRECTION)
        Kv_1 = np.array([[ 1,2,1]]).reshape(3,1)
        Kv_2 = np.array([[-1,0,1]]).reshape(1,3)


        ## PRE-ALLOCATE INTENSITY GRADIENTS
        G_up = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_vp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_rp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_tp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))


        ## CALCULATE IMAGE GRADIENTS
        for v_p in range(1, HEIGHT_PIXELS-1): 
            for u_p in range(1, WIDTH_PIXELS-1):
                G_up[v_p,u_p] = (Ku_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Ku_1)))).item()
                G_vp[v_p,u_p] = (Kv_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Kv_1)))).item()
                G_rp[v_p,u_p] = (2*(u_p - O_up) + 1)*G_up[v_p,u_p] + (2*(v_p - O_vp) + 1)*G_vp[v_p,u_p]
                G_tp[v_p,u_p] = cur_img[v_p,u_p] - prev_img[v_p,u_p]


        ## SOLVE LEAST SQUARES PROBLEM
        X = np.array([
            [f*np.sum(G_vp*G_vp), -f*np.sum(G_up*G_vp), -w/2*np.sum(G_rp*G_vp)],
            [f*np.sum(G_vp*G_up), -f*np.sum(G_up*G_up), -w/2*np.sum(G_rp*G_up)],
            [f*np.sum(G_vp*G_rp), -f*np.sum(G_up*G_rp), -w/2*np.sum(G_rp*G_rp)]
        ])

        y = np.array([
            [np.sum(G_tp*G_vp)],
            [np.sum(G_tp*G_up)],
            [np.sum(G_tp*G_rp)]
        ])*(8*w/delta_t)

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y
        b = b.flatten()

        return b

if __name__ == '__main__':

    OF = Optical_Flow(L=1,FPS=50)
    
    OF.Generate_Pattern(width=2,height=2,pixel_density=200)