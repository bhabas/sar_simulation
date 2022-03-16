import numpy as np
import pandas as pd
import getpass
import os
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import cv2 as cv

WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160

class DataParser:

    def __init__(self):

        ## FILEPATH PROPERTIES
        self.Filename = input('\nInput the name of the file to be parsed:\n')
        self.Username = getpass.getuser()
        self.Path = f'/home/{self.Username}/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Featureless_TTC/local_logs/'
        self.Filepath = self.Path + self.Filename

        ## CAMERA PROPERTIES
        self.w = 3.6e-6  # Pixel width [m]
        self.f = 0.66e-3/2 # Focal Length [m]
        self.O_x = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
        self.O_y = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]

        ## Check if data has already been compile
        pre_compiled_Flag = input('Is the data already compiled? (y/n): ')

        #check if the file exists and loop back if not
        if os.path.isfile(self.Filepath) and os.access(self.Filepath, os.R_OK):
            print("\nSuccessfully opened file\n")
        else:
            print(f"\nCould not access file named {self.Filename}\n")
            DataParser() #loop back so program doesn't exit from a typo

        #convert data into a useable format
        self.Data = pd.read_csv(self.Filepath,quotechar = '"',low_memory = False)
        self.Camera_splitter()
        #self.CameraData = np.fromstring(self.Data['Camera_Data'],str = ' ')
        self.Time = self.Data['Time'].to_numpy()
        self.Z_pos = self.Data['Z_pos'].to_numpy()
        self.Z_vel = self.Data['Z_vel'].to_numpy()
        self.X_vel = self.Data['X_vel'].to_numpy()
        self.Tau = self.Data['Tau'].to_numpy()
        self.OFx = self.Data['OFx'].to_numpy()
        self.OFy = self.Data['OFy'].to_numpy()

      



        if pre_compiled_Flag == 'y':

            self.TTC_est1 = self.Data['TTC_est1'].to_numpy()
            self.TTC_est2 = self.Data['TTC_est2'].to_numpy()
            self.OFx_est = self.Data['OFx_est'].to_numpy()
            self.OFy_est = self.Data['OFy_est'].to_numpy()

            self.Plotter()


        else: 
            self.Data_analysis()

            ## Append estimated tau values to avoid recalculating each time
            df_Tau = pd.DataFrame({
                'TTC_est1':self.TTC_est1, 
                'TTC_est2':self.TTC_est2,
                'OFx_est':self.OFx_est,
                'OFy_est':self.OFy_est,
                })
            pd.concat([self.Data.iloc[:,:-1],df_Tau,self.Data.iloc[:,-1]],axis=1).to_csv(self.Path+"Appended_"+self.Filename,index=False)

            self.Plotter()


        

        

    def Camera_splitter(self): # turn each string into an array

        self.Camera_data = self.Data["Camera_Data"].to_numpy()
        self.Camera_array = np.zeros((len(self.Camera_data), WIDTH_PIXELS*HEIGHT_PIXELS)) #leave the data as a 1D array, easier to index
        for n in range(0,self.Camera_data.size): #iterate through each row
            #make each row an array of ints
            self.Camera_array[n] = np.fromstring(self.Camera_data[n], dtype=int, sep = ' ')

    def Data_analysis(self):

        ## DEFINE PIXEL GRID
        u_grid = np.arange(0,WIDTH_PIXELS,1)
        v_grid = np.arange(0,HEIGHT_PIXELS,1)
        U_grid,V_grid = np.meshgrid(u_grid,v_grid)

        xi_grid = (U_grid - self.O_x)*self.w + self.w/2
        yi_grid = (V_grid - self.O_y)*self.w + self.w/2

        Ix = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        Iy = np.zeros_like(Ix)

        Prev_img = np.reshape(self.Camera_array[0],(HEIGHT_PIXELS,WIDTH_PIXELS)) #first image of the dataset 

        ## DEFINE KERNALS USED TO CALCULATE INTENSITY GRADIENTS
        Kx = 1/8*np.array([ # NORMALIZED SOBEL KERNAL (X-DIRECTION)
            [-1,0,1],
            [-2,0,2],
            [-1,0,1]]) 

        Ky = 1/8*np.array([ # NORMALIZED SOBEL KERNAL (Y-DIRECTION)
            [ 1, 2, 1],
            [ 0, 0, 0],
            [-1,-2,-1]]) 

        ## PRE-INITIALIZE SPACE FOR ESTIMATED VALUES
        self.TTC_est1 = np.zeros_like(self.Time)
        self.TTC_est2 = np.zeros_like(self.Time)
        self.OFx_est = np.zeros_like(self.Time)
        self.OFy_est = np.zeros_like(self.Time)

        for n in range(1,self.Camera_array.shape[0]): #start at the second data point
            
            Cur_img = np.reshape(self.Camera_array[n],(HEIGHT_PIXELS,WIDTH_PIXELS)) #reshape the array back into the image
            # plt.imshow(Cur_img, interpolation='none',cmap=cm.Greys)
            # plt.show()

            #gaussian smoothing the image
            # Cur_img = cv.GaussianBlur(Cur_img,(5,5),0)
            # plt.imshow(Cur_img, interpolation='none',cmap=cm.Greys)
            # plt.show()


            
            ## FIND IMAGE GRADIENTS
            for i in range(1,HEIGHT_PIXELS - 1): 
                for j in range(1,WIDTH_PIXELS - 1):
                    Ix[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Kx)/self.w
                    Iy[i,j] = -np.sum(Cur_img[i-1:i+2,j-1:j+2] * Ky)/self.w



            It = (Cur_img - Prev_img)/(self.Time[n] - self.Time[n-1]) #Brightness gradient
            G = xi_grid*Ix + yi_grid*Iy

            ## SOLVE LEAST SQUARES PROBLEM
            X = np.array([
                [np.sum(Ix**2),np.sum(Ix*Iy),np.sum(G*Ix)],
                [np.sum(Ix*Iy),np.sum(Iy**2),np.sum(G*Iy)],
                [np.sum(G*Ix),np.sum(G*Iy),np.sum(G**2)]
            ])

            y = -np.array([[np.sum(Ix*It)],[np.sum(Iy*It)],[np.sum(G*It)]])

            ## SOLVE b VIA PSEUDO-INVERSE
            b = np.linalg.pinv(X)@y

            self.OFy_est[n-1] = b[0,0]/self.f
            self.OFx_est[n-1] = b[1,0]/self.f
            self.TTC_est2[n-1] = 1/(b[2,0])



            # for i in range(1,HEIGHT_PIXELS-1): #Calculate Radial gradient G
            #     for j in range(1,WIDTH_PIXELS-1):
            #         Ix[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Kx)/self.w 
            #         Iy[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Ky)/self.w

            #CASE I
            # G = (xi_grid * Ix) + (yi_grid * Iy)
            # It = (Cur_img - Prev_img)/(self.Time[n] - self.Time[n-1]) #Brightness gradient
            # C = (-np.sum(G*It))/(np.sum(G*G)) #checking order of operations to see if it fixes how off it is
            #C = (-(np.sum(G)) * np.sum(It)) / (np.sum(G) ** 2)
            # self.TTC_est1[n-1] = 1/C

            # #CASE II
            # LHS = np.array([
            #     [np.sum(Ix*Ix),np.sum(Ix*Iy),np.sum(G*Ix)],
            #     [np.sum(Ix*Iy),np.sum(Iy*Iy),np.sum(G*Iy)],
            #     [np.sum(G*Ix),np.sum(G*Iy),np.sum(G*G)]
            # ])

            # RHS = np.array([-np.sum(Ix*It),-np.sum(Iy*It),-np.sum(G*It)])
            # ABC,_,_,_ = np.linalg.lstsq(LHS,RHS, rcond = None)
            # A = ABC[0]
            # B = ABC[1]
            # self.TTC_est2[n-1] = 1/(ABC[2])
            # self.OFx_est[n-1] = -B/self.f
            # self.OFy_est[n-1] = -A/self.f

            Prev_img = Cur_img

            print(f"Current Image: {n}/{self.Camera_array.shape[0]}")
                
    def Plotter(self): #plot results
        
        fig, ax = plt.subplots(3,1, sharex = True)
        ax[0].set_title("TTC Comparison")
        # ax[0].plot(self.Time,self.TTC_est1,'r', label = 'TTC_est1')
        ax[0].plot(self.Time,self.Tau,'b',label = 'TTC')
        ax[0].plot(self.Time,self.TTC_est2,'g',label = 'TTC_est2')
        ax[0].set_ylabel("TTC (seconds)")
        ax[0].set_ylim(-1,6)
        ax[0].grid()
        ax[0].legend(loc='upper right')

        ax[1].plot(self.Time,self.Z_pos,'g', label = 'Z position')
        ax[1].set_ylim(-1,2.5)
        ax[1].axhline(2.10,color='k',linestyle='dashed',label='Ceiling Height')
        ax[1].set_ylabel("Position (m)")
        ax[1].grid()
        ax[1].legend(loc='lower right')

        ax[2].plot(self.Time,self.Z_vel,color = 'black', label = 'Z velocity')
        ax[2].plot(self.Time,self.X_vel,color = 'r', label = 'X velocity')
        ax[2].set_ylim(-1,4.0)
        ax[2].set_ylabel("Velocity (m)")
        ax[2].set_xlabel("Time") 
        ax[2].grid()
        ax[2].legend(loc='upper right')


        #plt.figure(2)
        fig2 , ax2 = plt.subplots(2,1, sharex = True)
        ax2[0].set_title('Optical Flow Comparison')
        ax2[0].plot(self.Time,self.OFy, color = 'black', label = 'OFy')
        ax2[0].plot(self.Time,self.OFy_est, color = 'r',label = 'OFy_est')
        ax2[0].set_ylim(-1,10.0)
        ax2[0].grid()
        ax2[0].legend()

        ax2[1].plot(self.Time,self.OFx, color = 'black',label = 'OFx')
        ax2[1].plot(self.Time,self.OFx_est, color = 'b',label = 'OFx_est')
        ax2[1].set_xlabel("Time")
        ax2[1].set_ylabel('Optical Flow')
        ax2[1].set_ylim(-1,10.0)
        ax2[1].grid()
        ax2[1].legend()

        plt.show()


if __name__ == '__main__':

    DataParser() #init class