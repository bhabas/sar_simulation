from configparser import Interpolation
import rospy
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import sys

from example_msgs.msg import CustomMessage
Pixel_Height = 160
Pixel_Width = 160

class DataCheck:

    def __init__(self):


        #INIT VARS
        self.Ku = 1/8*np.array([
            [-1, 0, 1],
            [-2, 0, 2],
            [-1, 0 ,1]
        ])

        self.Kv = 1/8*np.array([
            [-1,-2,-1],
            [ 0, 0, 0],
            [ 1, 2, 1]
        ])

        # self.img = np.array([0,0,0,0,0,0,0,8,8,8,8,0,0,8,0,0,8,0,0,8,0,0,8,0,0,8,8,8,8,0,0,0,0,0,0,0]).reshape(Pixel_Height,Pixel_Width)
        # self.prev_img= np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,8,0,0,0,0,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0]).reshape(Pixel_Height,Pixel_Width) 
        # print(self.img)

        self.w = 3.6e-6
        self.f = 3.3e-4

        self.Iu = np.zeros((Pixel_Height,Pixel_Width))
        self.Iv = np.zeros_like(self.Iu)
        self.It = np.zeros_like(self.Iu)
        self.prev_time = 0

        # self.Iu = np.zeros((5,5))
        # self.Iv = np.zeros_like(self.Iu)

        rospy.init_node('DataChecker',anonymous=True)
        
        np.set_printoptions(threshold = sys.maxsize) #allows it to print the full string without truncation

        rospy.Subscriber("/MyPup_cpp",CustomMessage,self.Convolution_XY, queue_size = 1)
        # msg = rospy.wait_for_message("/MyPub_cpp",CustomMessage,timeout = None)
        # self.DataCheck_cb(msg)


    def Convolution_XY(self,Cam_msg):

        self.t = np.round(Cam_msg.header.stamp.to_sec(),4)
        self.Cur_img = np.frombuffer(Cam_msg.data, np.uint8,).reshape(Pixel_Width,Pixel_Height)
        self.Prev_img = np.frombuffer(Cam_msg.data, np.uint8,).reshape(Pixel_Width,Pixel_Height)

        # NP CONVOLUTION WE HAVE BEEN USING
        for i in range(1,Pixel_Height - 1):
            for j in range(1,Pixel_Width - 1):
                self.Iu[i,j] = np.sum(self.Cur_img[i-1:i+2,j-1:j+2] * self.Ku)/self.w
                self.Iv[i,j] = np.sum(self.Cur_img[i-1:i+2,j-1:j+2] * self.Kv)/self.w
                self.It[i,j] = (self.Cur_img[i,j] - self.Prev_img[i,j])/(self.t - self.prev_time)

        self.prev_time = self.t
        self.Math()

    def Math(self): #Checking the final matrices to make sure they match

        #Setting constants
        w = self.w
        f = self.f
        O_up = Pixel_Width/2    # Pixel X_offset [pixels]
        O_vp = Pixel_Height/2   # Pixel Y_offset [pixels]

        ## DEFINE PIXEL GRID
        up_grid = np.arange(0,Pixel_Width,1)
        vp_grid = np.arange(0,Pixel_Height,1)
        Up_grid,Vp_grid = np.meshgrid(up_grid,vp_grid)

        ## DEFINE IMAGE SENSOR COORDS
        U_grid = (Up_grid - O_up)*w + w/2 #GENERALIZE THE VGRID AND UGRID IN C++
        V_grid = (Vp_grid - O_vp)*w + w/2

        G = U_grid*self.Iu + V_grid*self.Iv # Radial Gradient

        X = np.array([
                [f*np.sum(self.Iu**2),      f*np.sum(self.Iu*self.Iv),  np.sum(G*self.Iu)],
                [f*np.sum(self.Iu*self.Iv), f*np.sum(self.Iv**2),       np.sum(G*self.Iv)],
                [f*np.sum(G*self.Iu),       f*np.sum(G*self.Iv),        np.sum(G**2)]
        ])

        y = np.array([
                [-np.sum(self.Iu*self.It)],
                [-np.sum(self.Iv*self.It)],
                [-np.sum(G*self.It)]
        ])

        print(f"\nLHS:\n{X}")
        print(f"\nRHS:\n{y}\n")        

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y
        print(b)

        self.TTC_est = 1/(b[2])

        print(f"\nTTC estimate: {self.TTC_est}\n")


    def Comparator(self):

        #Y PLOT
        fig,ax = plt.subplots(4,1, sharex = False)
        ax[0].set_title("Image Comparison Y")
        ax[0].imshow(self.Cur_Img, interpolation = "none",cmap = cm.Greys)

        # ax[1].imshow(self.Convy,interpolation = "none",cmap = cm.Greys)
        # ax[1].set_ylabel("Conv Y")

        # Yu = self.Convy - self.Iu
        ax[2].imshow(self.grad_y,interpolation = "none",cmap = cm.Greys)
        ax[2].set_ylabel("grad y")

        # Yx = self.Convy - self.Iv
        ax[3].imshow(self.Iv,interpolation = "none",cmap = cm.Greys)
        ax[3].set_ylabel("Iv")

        #X PLOT
        fig2, ax2 = plt.subplots(4,1)
        ax2[0].set_title("Image Comparison X")
        ax2[0].imshow(self.Cur_Img, interpolation = "none", cmap = cm.Greys)

        # ax2[1].imshow(self.Convx, interpolation = "none",cmap = cm.Greys)
        # ax2[1].set_ylabel("Conv X")

        ax2[2].imshow(self.Iu, interpolation = "none",cmap = cm.Greys)
        ax2[2].set_ylabel("Iu")

        ax2[3].imshow(self.grad_x, interpolation = "none", cmap = cm.Greys)
        ax2[3].set_ylabel("grad x")

        plt.show()


if __name__=='__main__':

    DataCheck()
    rospy.spin()