import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
import cv2 as cv
from sensor_msgs.msg import Image #import camera data
from crazyflie_msgs.msg import CF_StateData #import 'true' values

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160


class CameraClass:

    def __init__(self):

        rospy.init_node('Camera_alg', anonymous = True)#start nodes

        # Pre_init vars
        self.Tau_act_List = [0]
        self.Tau_est_List = [0]
        self.Tau_cpp_est_List = [0]
        self.OFy_act_List = [0]
        self.OFy_ext_List = [0]
        self.OFx_act_List = [0]
        self.OFx_ext_List = [0]
        self.t_List = [0]

        self.Cur_img = np.array([])
        self.Prev_img = np.zeros([WIDTH_PIXELS,HEIGHT_PIXELS])
        self.t = 0
        self.Tau = 0
        self.OFx = 0
        self.OFy = 0
        self.Tau_est_cpp = 0
        self.d_ceil = 0
        self.prev_time = 0

        self.cam_sub = rospy.Subscriber("/CF_Internal/camera/image_raw",Image,self.Camera_cb,queue_size = 1)
        # self.state_sub = rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size = 1)
            

    def Camera_cb(self,Cam_msg):

        self.time = np.round(Cam_msg.header.stamp.to_sec(),6) #grabbing timestamp
        self.Cur_img = np.frombuffer(Cam_msg.data,np.uint8).reshape(WIDTH_PIXELS,HEIGHT_PIXELS) #pulling in image to process

        self.cam_alg() #once the images are updated time to process them

        # if(self.t != self.prev_time):
        #     self.cam_alg(self)
       

    def CF_StateDataCallback(self,StateData_msg):
        
        self.Tau = np.round(StateData_msg.Tau,3)
        self.OFx = np.round(StateData_msg.OFx,3)
        self.OFy = np.round(StateData_msg.OFy,3)
        self.Tau_est_cpp = np.round(StateData_msg.Tau_est,3)
        self.d_ceil = np.round(StateData_msg.D_ceil,3)


    def cam_alg(self):

        O_up = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
        O_vp = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]
        w = 3.6e-6              # Pixel width [m]
        f = 0.66e-3/2           # Focal Length [m]

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

        ## FIND IMAGE GRADIENTS
        for i in range(1,HEIGHT_PIXELS - 1): 
            for j in range(1,WIDTH_PIXELS - 1):
                Iu[i,j] = np.sum(self.Cur_img[i-1:i+2,j-1:j+2] * Kx)/w
                Iv[i,j] = np.sum(self.Cur_img[i-1:i+2,j-1:j+2] * Ky)/w

        It = (self.Cur_img - self.Prev_img)/(self.time - self.prev_time) # Time Gradient

        print(f"Change in time: {(self.time - self.prev_time):.4f}") #checking to make sure dt is 1/120

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

        self.Prev_img = self.Cur_img #set Previous image to Current one before reiterating 
        self.prev_time = self.time
        self.logs()

    
    def logs(self):


        Tau_act = self.Tau
        OFy_act = self.OFy
        OFx_act = self.OFx
        
        print(f"Tau_act: {Tau_act:.3f} | Tau_est: {1/self.b[2]:.3f} | Tau_est_cpp: {self.Tau_est_cpp}")
        print(f"OFy_act: {OFy_act:.3f} | OFy_est: {self.b[0]:.3f}")
        print(f"OFx_act: {OFx_act:.3f} | OFx_est: {self.b[1]:.3f}")
        print(f"Distance to Ceiling {self.d_ceil:.3f}\n")


        ## APPEN OPTICAL FLOW ESTIMATES TO LIST FOR PLOTTING
        self.Tau_est_List.append(1/self.b[2])
        self.Tau_act_List.append(Tau_act)
        self.Tau_cpp_est_List.append(self.Tau_est_cpp)
        self.OFy_act_List.append(OFy_act)
        self.OFy_ext_List.append(self.b[0])
        self.OFx_act_List.append(OFx_act)
        self.OFx_ext_List.append(self.b[1])
        self.t_List.append(self.t)

        # if (self.d_ceil < 0.1):
        #         self.LoggingFlag = False
        #         self.plotter()

    def plotter(self):
        
        ## TAU PLOT
        plt.plot(self.t_List,self.Tau_est_List,'rx',label="Tau_estimate")
        plt.plot(self.t_List,self.Tau_act_List,label="Tau_actual")
        plt.plot(self.t_List,self.Tau_cpp_est_List,label = "Tau_cpp_estimate")
        plt.title('Tau Estimation')
        plt.xlabel('Time [s]')
        plt.ylabel('Tau [s]')
        plt.grid()
        plt.legend()
        plt.show()


if __name__ == '__main__':

    CameraClass() #initialize the class when run
    rospy.spin() #run until this program is shutdown
