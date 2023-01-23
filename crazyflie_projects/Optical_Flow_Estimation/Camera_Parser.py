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

    def Plot_Image(self,n=0):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        ax.imshow(self.Camera_array[n].reshape(WIDTH_PIXELS,HEIGHT_PIXELS), interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',)

        plt.show()



if __name__ == '__main__':

    Parser = DataParser() #init class
    Parser.Plot_Image()