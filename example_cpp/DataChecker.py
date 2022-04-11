from configparser import Interpolation
import rospy
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from example_msgs.msg import CustomMessage
Pixel_Height = 160
Pixel_Width = 120

class DataCheck:

    def __init__(self):


        #INIT VARS
        self.Ku = np.array([
            [-1, 0, 1],
            [-2, 0, 2],
            [-1, 0 ,1]
        ])

        self.Kv = np.array([
            [-1,-2,-1],
            [ 0, 0, 0],
            [ 1, 2, 1]
        ])
        
        self.Iu = np.zeros((Pixel_Width,Pixel_Height))
        self.Iv = np.zeros_like(self.Iu)

        rospy.init_node('DataChecker',anonymous=True)

        msg = rospy.wait_for_message("/MyPub_cpp",CustomMessage,timeout = None)
        self.DataCheck_cb(msg)


    def Convolution(self):

        # NP CONVOLUTION WE HAVE BEEN USING
        for i in range(1,Pixel_Width - 1):
            for j in range(1,Pixel_Height - 1):
                self.Iu[i,j] = np.sum(self.Image[i-1:i+2,j-1:j+2] * self.Ku)
                self.Iv[i,j] = np.sum(self.Image[i-1:i+2,j-1:j+2] * self.Kv)

        # TESTING OPEN CV FUNCTIONS

        # img = cv.cvtColor(self.Image, cv.COLOR_BGR2GRAY)
        self.grad_x = cv.Sobel(self.Image,cv.CV_16S,1,0,ksize = 3,delta = 0)
        self.grad_y = cv.Sobel(self.Image,cv.CV_16S,0,1,ksize = 3,delta = 0)

        self.Comparator()


    def DataCheck_cb(self,Data):

        self.Image = np.frombuffer(Data.Camera_data, np.uint8).reshape(120,160)#create as 160 x 160
        # self.Image = np.delete(self.Image, slice(120,160), axis = 1) # now remove the last 40 rows
        self.Convy = np.frombuffer(Data.Yconv,np.uint8).reshape(120,160)
        #self.Convy = np.delete(self.Convy, np.slice(120,160), axis = 1)
        self.Convx = np.frombuffer(Data.Xconv,np.uint8).reshape(120,160)
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