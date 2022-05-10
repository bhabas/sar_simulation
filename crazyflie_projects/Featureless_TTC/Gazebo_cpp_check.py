import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
import cv2 as cv
from sensor_msgs.msg import Image #import camera data
from crazyflie_msgs.msg import CF_StateData #import 'true' values

# Pre_init vars

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160
FPS = 120               # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3/2           # Focal Length [m]
O_up = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_vp = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]

## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.25    # [m]

d_0 = 0.6   # Initial Camera height [m]
vz = 0.7    # Camera velocity [m/s]
vx = 0.3
vy = 0.3

Tau_act_List = []
Tau_est_List = []
Tau_cpp_est_List = []
OFy_act_List = []
OFy_ext_List = []
OFx_act_List = []
OFx_ext_List = []
t_List = []


class CameraClass:

    def __init__(self):

        rospy.init_node('Camera_alg', anonymous = True)#start nodes

        self.Cur_img = np.array([])
        self.Prev_img = np.zeros([WIDTH_PIXELS,HEIGHT_PIXELS])
        rospy.Subscriber("/CF_Internal/camera/image_raw",Image,self.Camera_cb,queue_size = 1)
        rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size = 1)

    def Camera_cb(self,Cam_msg):
        self.t = rospy.get_time()
        self.Cur_img = np.frombuffer(Cam_msg.data,np.uint8).reshape(WIDTH_PIXELS,HEIGHT_PIXELS)
        self.cam_alg(self.Cur_img,self.Prev_img)  

    def CF_StateDataCallback(self,StateData_msg):
        
        self.Tau = np.round(StateData_msg.Tau,3)
        self.OFx = np.round(StateData_msg.OFx,3)
        self.OFy = np.round(StateData_msg.OFy,3)
        self.Tau_est_cpp = np.round(StateData_msg.Tau_est,3)
        self.d_ceil = np.round(StateData_msg.D_ceil,3)


    ## PIXEL INTENSITIES/GRADIENTS
    def I_continuous(u_p,z_0,t):
        ## CONVERT PIXEL INDEX TO METERS
        u = (u_p - O_up)*w + w/2 
        d = z_0+vz*t

        ## RETURN AVERAGE BRIGHTNESS VALUE OVER PIXEL
        return I_0/2 * np.sin(2*np.pi*(u*d/(f*L) + vx*t/L)) + I_0/2


    def cam_alg(self,Cur_img,Prev_img):


        ## PRE-ALLOCATE IMAGE ARRAY [pixels]
        u_p = np.arange(0,WIDTH_PIXELS,1)
        v_p = np.arange(0,HEIGHT_PIXELS,1)
        U_p,V_p = np.meshgrid(u_p,v_p)

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

        # Cur_img = cv.GaussianBlur(Cur_img,(5,5),0)
        # Prev_img = cv.GaussianBlur(Prev_img,(5,5),0)

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
        self.b = np.linalg.pinv(X)@y
        self.b = self.b.flatten()

        self.Prev_img = Cur_img #set Previous image to Current one before reiterating 
        self.animate_func()

    
    def animate_func(self):


        Tau_act = self.Tau
        OFy_act = self.OFy
        OFx_act = self.OFx
        
        print(f"Tau_act: {Tau_act:.3f} | Tau_est: {1/self.b[2]:.3f} | Tau_est_cpp: {self.Tau_est_cpp}")
        print(f"OFy_act: {OFy_act:.3f} | OFy_est: {self.b[0]:.3f}")
        print(f"OFx_act: {OFx_act:.3f} | OFx_est: {self.b[1]:.3f}\n")


        ## APPEN OPTICAL FLOW ESTIMATES TO LIST FOR PLOTTING
        Tau_est_List.append(1/self.b[2])
        Tau_act_List.append(Tau_act)
        Tau_cpp_est_List.append(self.Tau_est_cpp)
        OFy_act_List.append(OFy_act)
        OFy_ext_List.append(self.b[0])
        OFx_act_List.append(OFx_act)
        OFx_ext_List.append(self.b[1])

        t_List.append(self.t)

        if (self.d_ceil < 0.1):
            self.plotter()

    def plotter(self):
        ## TAU PLOT
        fig2 = plt.figure(1)
        ax = fig2.add_subplot(111)

        ax.plot(t_List,Tau_est_List,'rx',label="Tau_estimate")
        ax.plot(t_List,Tau_act_List,label="Tau_actual")
        ax.plot(t_List,Tau_cpp_est_List,label = "Tau_cpp_estimate")
        ax.set_title('Tau Estimation')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Tau [s]')
        ax.grid()
        ax.legend()

        fig2.tight_layout()


if __name__ == '__main__':

    CameraClass() #initialize the class when run
    rospy.spin() #run until this program is shutdown
