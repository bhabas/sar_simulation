import numpy as np
import pandas as pd
import getpass
import os
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import cv2 as cv


## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160
FPS = 60               # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3/2          # Focal Length [m]
O_up = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_vp = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]

FILTER_FLAG = False


class DataParser:

    def __init__(self):

        ## INIT LOGGING PARAMETERS
        self.Username = getpass.getuser()
        self.DirPath = f'/home/{self.Username}/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Optical_Flow_Estimation/local_logs' 
        # self.FileName = input("Input the name of the log file:\n")
        self.FileName = "ExampleLog.csv"
        self.FilePath = os.path.join(self.DirPath,self.FileName)

 
        ## CHECK IF DATA HAS ALREADY BEEN COMPILED
        # pre_compiled_Flag = input('Is the data already compiled? (y/n): ')

        ## PARSE CSV DATA
        self.Data_df = pd.read_csv(self.FilePath,quotechar = '"',low_memory = False)
        self.t = self.Data_df['t'].to_numpy()
        self.x = self.Data_df['x'].to_numpy()
        self.y = self.Data_df['y'].to_numpy()
        self.z = self.Data_df['z'].to_numpy()

        self.vx = self.Data_df['vx'].to_numpy()
        self.vy = self.Data_df['vy'].to_numpy()
        self.vz = self.Data_df['vz'].to_numpy()

        self.wx = self.Data_df['wx'].to_numpy()
        self.wy = self.Data_df['wy'].to_numpy()
        self.wz = self.Data_df['wz'].to_numpy()

        self.qx = self.Data_df['qx'].to_numpy()
        self.qy = self.Data_df['qy'].to_numpy()
        self.qz = self.Data_df['qz'].to_numpy()
        self.qw = self.Data_df['qw'].to_numpy()


        self.D_perp = self.Data_df['D_perp'].to_numpy()
        self.Tau = self.Data_df['Tau'].to_numpy()
        self.Theta_x = self.Data_df['Theta_x'].to_numpy()
        self.Theta_y = self.Data_df['Theta_y'].to_numpy()

        self.Camera_data = self.Data_df["Camera_Data"].to_numpy()

        ## CONVERT IMAGE DATA FROM STRING TO INTEGER
        self.Camera_array = np.zeros((len(self.Camera_data), WIDTH_PIXELS*HEIGHT_PIXELS)) # Allocate space for camera data
        for n in range(0,self.Camera_data.size): # Make each row an array of ints
            self.Camera_array[n] = np.fromstring(self.Camera_data[n], dtype=int, sep =' ')

    def grabImage(self,idx):

        return self.Camera_array[idx].reshape(WIDTH_PIXELS,HEIGHT_PIXELS)

    def grabState(self,state_str,idx):

        return self.Data_df[state_str][idx]


    def Plot_Image(self,image_array):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        ax.imshow(image_array.reshape(WIDTH_PIXELS,HEIGHT_PIXELS), interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',)

        plt.show()

    def OpticalFlow_Calc(self,cur_img,prev_img,delta_t):


        ## DEFINE KERNALS USED TO CALCULATE INTENSITY GRADIENTS
        Ku = np.array([ # SOBEL KERNAL (U-DIRECTION)
            [-1,-2,-1],
            [ 0, 0, 0],
            [ 1, 2, 1]
        ]) 

        Kv = np.array([ # SOBEL KERNAL (V--DIRECTION)
            [ -1, 0, 1],
            [ -2, 0, 2],
            [ -1, 0, 1]
        ])

        
        ## PRE-ALLOCATE IMAGE ARRAY [pixels]
        v_p = np.arange(0,WIDTH_PIXELS,1)
        u_p = np.arange(0,HEIGHT_PIXELS,1)
        V_p,U_p = np.meshgrid(v_p,u_p)

        ## DEFINE IMAGE SENSOR COORDS IN [m]
        V_grid = -((V_p - O_vp)*w + w/2)
        U_grid =  ((U_p - O_up)*w + w/2)

        ## PRE-ALLOCATE INTENSITY GRADIENTS
        Iu = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        Iv = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))


        ## CALCULATE IMAGE GRADIENTS
        for ii in range(1, HEIGHT_PIXELS-1): 
            for jj in range(1, WIDTH_PIXELS-1):
                Iu[ii,jj] = 1/(8*w)*np.sum(cur_img[ii-1:ii+2,jj-1:jj+2] * Ku)
                Iv[ii,jj] = -1/(8*w)*np.sum(cur_img[ii-1:ii+2,jj-1:jj+2] * Kv)

        ## CALCULATE TIME GRADIENT AND RADIAL GRADIENT
        It = (cur_img - prev_img)/delta_t   # Time gradient
        Ir = U_grid*Iu + V_grid*Iv          # Radial Gradient


        ## SOLVE LEAST SQUARES PROBLEM
        X = np.array([
            [f*np.sum(Iu*Iu), f*np.sum(Iv*Iu), -np.sum(Ir*Iu)],
            [f*np.sum(Iu*Iv), f*np.sum(Iv*Iv), -np.sum(Ir*Iv)],
            [f*np.sum(Iu*Ir), f*np.sum(Iv*Ir), -np.sum(Ir*Ir)]
        ])

        y = np.array([
            [np.sum(It*Iu)],
            [np.sum(It*Iv)],
            [np.sum(It*Ir )]
        ])

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y
        b = b.flatten()


        #     self.OFy_est[n] = b[0]
        #     self.OFx_est[n] = b[1]
        #     self.Tau_est[n] = 1/(b[2])

        #     Prev_img = Cur_img

        return b



if __name__ == '__main__':

    Parser = DataParser() #init class

    idx = 1
    prev_img = Parser.grabImage(idx-1)
    cur_img = Parser.grabImage(idx)

    t_prev = Parser.grabState('t',idx-1)
    t_cur = Parser.grabState('t',idx)

    delta_t = t_cur-t_prev


    # Parser.Plot_Image(img)
    # Parser.Plot_Image()
    b = Parser.OpticalFlow_Calc(cur_img,prev_img,delta_t)
    print(b[2]**-1)