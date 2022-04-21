from configparser import Interpolation
import rospy
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import sys

from example_msgs.msg import CustomMessage
Pixel_Height = 5
Pixel_Width = 5

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

        self.img = np.array([0,0,0,0,0,0,0,8,8,8,8,0,0,8,0,0,8,0,0,8,0,0,8,0,0,8,8,8,8,0,0,0,0,0,0,0]).reshape(5,5)
        self.prev_img= np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,8,0,0,0,0,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0]).reshape(5,5) 
        print(self.img)

        # self.Iu = np.zeros((5,5))
        # self.Iv = np.zeros_like(self.Iu)

        # rospy.init_node('DataChecker',anonymous=True)
        
        # np.set_printoptions(threshold = sys.maxsize) #allows it to print the full string without truncation

        # msg = rospy.wait_for_message("/MyPub_cpp",CustomMessage,timeout = None)
        # self.DataCheck_cb(msg)
        self.Check()

    def Check(self):

        # self.Image = np.frombuffer(Data.Camera_data, np.uint8).reshape(5,5)
        # self.Convy = np.array(Data.Yconv).reshape(5,5)
        # self.Convx = np.array(Data.Xconv).reshape(5,5)

        w = 3.6e-6
        f = 3.3e-4
        O_up = Pixel_Width/2    # Pixel X_offset [pixels]
        O_vp = Pixel_Height/2   # Pixel Y_offset [pixels]

        ## DEFINE PIXEL GRID
        up_grid = np.arange(0,Pixel_Width,1)
        vp_grid = np.arange(0,Pixel_Height,1)
        Up_grid,Vp_grid = np.meshgrid(up_grid,vp_grid)

        ## DEFINE IMAGE SENSOR COORDS
        U_grid = (Up_grid - O_up)*w + w/2 #GENERALIZE THE VGRID AND UGRID IN C++
        V_grid = (Vp_grid - O_vp)*w + w/2
        # print(f"U_grid:\n{U_grid}")
        # print(f"V_grid:\n{V_grid}")

        Iu = np.zeros((Pixel_Height,Pixel_Width))
        Iv = np.zeros((Pixel_Height,Pixel_Width))


        for i in range(1,Pixel_Height - 1):
            for j in range(1,Pixel_Width - 1):
                Iu[i,j] = np.sum(self.img[i-1:i+2,j-1:j+2] * self.Ku)/w
                Iv[i,j] = np.sum(self.img[i-1:i+2,j-1:j+2] * self.Kv)/w
                print(np.sum(self.img[i-1:i+2,j-1:j+2] * self.Ku)/w)

        It = self.img - self.prev_img #assuming dt is 1
        # print(f"\nIu:\n{Iu}")

        G = U_grid*Iu + V_grid*Iv # Radial Gradient
        print(f"\nIG:\n{(G)}")

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

        print(f"\nLHS:\n{X}")
        print(f"\nRHS:\n{y}")

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y
        print(f"\nX:\n{b}")
        b = b.flatten()

        self.OFy_est = b[0]
        self.OFx_est = b[1]
        self.TTC_est = 1/(b[2])

        print(f"\nOFy estimate: {self.OFy_est}\n")
        print(f"\nOFx estimate: {self.OFx_est}\n")
        print(f"\nTTC estimate: {self.TTC_est}\n")

        # ====== DEBUGGING ======

        # print(self.Image)
        # print("Iu:")
        # print(self.Iu)
        # print("\nIv:")
        # print(self.Iv)
        # print("Ix:")
        # print(self.Convx)
        # print("\nIy:")
        # print(self.Convy)


    def Convolution(self):

        # NP CONVOLUTION WE HAVE BEEN USING
        for i in range(1,Pixel_Width - 1):
            for j in range(1,Pixel_Width - 1):
                self.Iu[i,j] = np.sum(self.Image[i-1:i+2,j-1:j+2] * self.Ku)
                self.Iv[i,j] = np.sum(self.Image[i-1:i+2,j-1:j+2] * self.Kv)

        # TESTING OPEN CV FUNCTIONS

        # img = cv.cvtColor(self.Image, cv.COLOR_BGR2GRAY)
        self.grad_x = cv.Sobel(self.Image,cv.CV_16S,1,0,ksize = 3,delta = 0)
        self.grad_y = cv.Sobel(self.Image,cv.CV_16S,0,1,ksize = 3,delta = 0)

        self.Comparator()
        # self.Math()

    def Math(self):

        flatU = self.Iu.flatten()
        flatX = self.Convx.flatten()
        # for i in range(Pixel_Height*Pixel_Width):
        print(flatX - flatU)
        print("\n\n")
        # print(flatU - self.grad_x.flatten())


    def DataCheck_cb(self,Data):

        self.Image = np.frombuffer(Data.Camera_data, np.uint8).reshape(160,160)#create as 120rows x 160col
        # self.Image = np.delete(self.Image, slice(120,160), axis = 1) # now remove the last 40 rows
        self.Convy = np.array(Data.Yconv).reshape(160,160) #np.frombuffer(Data.Yconv,np.int8).reshape(120,160)
        #self.Convy = np.delete(self.Convy, np.slice(120,160), axis = 1)
        self.Convx = np.array(Data.Xconv).reshape(160,160)#np.frombuffer(Data.Xconv,np.int8).reshape(120,160)
        #self.Convx = np.delete(self.Convx, np.slice(120,160), axis = 1)
        
        self.Convolution()
        # print(self.Convx)

    def Comparator(self):

        #Y PLOT
        fig,ax = plt.subplots(4,1, sharex = False)
        ax[0].set_title("Image Comparison Y")
        ax[0].imshow(self.Image, interpolation = "none",cmap = cm.Greys)

        ax[1].imshow(self.Convy,interpolation = "none",cmap = cm.Greys)
        ax[1].set_ylabel("Conv Y")

        # Yu = self.Convy - self.Iu
        ax[2].imshow(self.grad_y,interpolation = "none",cmap = cm.Greys)
        ax[2].set_ylabel("grad y")

        # Yx = self.Convy - self.Iv
        ax[3].imshow(self.Iv,interpolation = "none",cmap = cm.Greys)
        ax[3].set_ylabel("Iv")

        #X PLOT
        fig2, ax2 = plt.subplots(4,1)
        ax2[0].set_title("Image Comparison X")
        ax2[0].imshow(self.Image, interpolation = "none", cmap = cm.Greys)

        ax2[1].imshow(self.Convx, interpolation = "none",cmap = cm.Greys)
        ax2[1].set_ylabel("Conv X")

        ax2[2].imshow(self.Iu, interpolation = "none",cmap = cm.Greys)
        ax2[2].set_ylabel("Iu")

        ax2[3].imshow(self.grad_x, interpolation = "none", cmap = cm.Greys)
        ax2[3].set_ylabel("grad x")

        plt.show()


if __name__=='__main__':

    DataCheck()
    # rospy.spin()