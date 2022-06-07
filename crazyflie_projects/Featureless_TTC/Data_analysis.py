import numpy as np
import pandas as pd
import getpass
import os
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import cv2 as cv

# import cv2 as cv

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160
FPS = 60               # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3/2          # Focal Length [m]
O_up = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_vp = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]


class DataParser:

    def __init__(self):

        ## FILEPATH PROPERTIES
        filename = input('\nInput the name of the file to be parsed:\n')
        self.Username = getpass.getuser()
        self.Path = f'/home/{self.Username}/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Featureless_TTC/logs/'
        filepath = self.Path + filename

        ## CHECK IF FILE EXISTS
        if os.path.isfile(filepath) and os.access(filepath, os.R_OK):
            print("\nSuccessfully opened file\n")
        else:
            print(f"\nCould not access file named {filename}\n")
            DataParser() #loop back so program doesn't exit from a typo

 
        ## CHECK IF DATA HAS ALREADY BEEN COMPILED
        pre_compiled_Flag = input('Is the data already compiled? (y/n): ')

        ## PARSE CSV DATA
        self.Data_df = pd.read_csv(filepath,quotechar = '"',low_memory = False)
        self.Time = self.Data_df['Time'].to_numpy()
        self.Z_pos = self.Data_df['Z_pos'].to_numpy()
        self.Z_vel = self.Data_df['Z_vel'].to_numpy()
        self.X_vel = self.Data_df['X_vel'].to_numpy()
        self.Tau = self.Data_df['Tau'].to_numpy()
        self.OFx = self.Data_df['OFx'].to_numpy()
        self.OFy = self.Data_df['OFy'].to_numpy()
        self.Camera_data = self.Data_df["Camera_Data"].to_numpy()

        ## CONVERT IMAGE DATA FROM STRING TO INTEGER
        self.Camera_array = np.zeros((len(self.Camera_data), WIDTH_PIXELS*HEIGHT_PIXELS)) # Allocate space for camera data
        for n in range(0,self.Camera_data.size): # make each row an array of ints
            self.Camera_array[n] = np.fromstring(self.Camera_data[n], dtype=int, sep = ' ')


        if pre_compiled_Flag == 'y':

            self.TTC_est = self.Data_df['Tau_est'].to_numpy()
            self.OFx_est = self.Data_df['OFx_est'].to_numpy()
            self.OFy_est = self.Data_df['OFy_est'].to_numpy()

            self.Plotter()


        else: 
            self.Data_analysis()

            ## Append estimated tau values to avoid recalculating each time
            df_Tau = pd.DataFrame({
                'Tau_est':self.Tau_est, 
                'OFx_est':self.OFx_est,
                'OFy_est':self.OFy_est,
                })
            pd.concat([self.Data_df.iloc[:,:-1],df_Tau,self.Data_df.iloc[:,-1]],axis=1).to_csv(self.Path+"Appended_"+filename,index=False)

            self.Plotter()



    def Data_analysis(self):

        ## DEFINE KERNALS USED TO CALCULATE INTENSITY GRADIENTS
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

        
        ## PRE-ALLOCATE IMAGE ARRAY [pixels]
        u_p = np.arange(0,WIDTH_PIXELS,1)
        v_p = np.arange(0,HEIGHT_PIXELS,1)
        U_p,V_p = np.meshgrid(u_p,v_p)

        ## DEFINE IMAGE SENSOR COORDS IN [m]
        U_grid = (U_p - O_up)*w + w/2
        V_grid = (V_p - O_vp)*w + w/2

        ## PRE-ALLOCATE INTENSITY GRADIENTS
        Iu = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        Iv = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))

        Iu_s = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        Iv_s = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))

        ## INITIALIZE PREVIOUS IMAGE
        Prev_img = np.reshape(self.Camera_array[0],(HEIGHT_PIXELS,WIDTH_PIXELS)) 
        Prev_img_smooth = np.reshape(self.Camera_array[0],(HEIGHT_PIXELS,WIDTH_PIXELS))
        Prev_img_smooth = cv.GaussianBlur(Prev_img_smooth,(5,5),0)


        ## PRE-INITIALIZE ARRAYS FOR LOGGING ESTIMATES
        self.Tau_est = np.zeros_like(self.Time)
        self.OFx_est = np.zeros_like(self.Time)
        self.OFy_est = np.zeros_like(self.Time)

        self.Tau_est_s = np.zeros_like(self.Time)
        self.OFx_est_s = np.zeros_like(self.Time)
        self.OFy_est_s = np.zeros_like(self.Time)

        for n in range(1,self.Camera_array.shape[0]): # Start at second data point to grab the previous image
            
            Cur_img = np.reshape(self.Camera_array[n],(HEIGHT_PIXELS,WIDTH_PIXELS)) #reshape the array back into the image
            Cur_img_smooth = cv.GaussianBlur(Cur_img,(5,5),0)


            ## FIND IMAGE GRADIENTS FOR UNFILT IMG
            for i in range(1,HEIGHT_PIXELS - 1): 
                for j in range(1,WIDTH_PIXELS - 1):
                    Iu[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Kx)/w
                    Iv[i,j] = np.sum(Cur_img[i-1:i+2,j-1:j+2] * Ky)/w

            ## CALCULATE TIME GRADIENT AND RADIAL GRADIENT
            It = (Cur_img - Prev_img)/(self.Time[n] - self.Time[n-1]) # Time gradient
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
            b = np.linalg.pinv(X)@y
            b = b.flatten()


            self.OFy_est[n] = b[0]
            self.OFx_est[n] = b[1]
            self.Tau_est[n] = 1/(b[2])

            Prev_img = Cur_img

            ################ SMOOTHED IMAGE ################


            ## FIND IMAGE GRADIENTS FOR UNFILT IMG
            for i in range(1,HEIGHT_PIXELS - 1): 
                for j in range(1,WIDTH_PIXELS - 1):
                    Iu_s[i,j] = np.sum(Cur_img_smooth[i-1:i+2,j-1:j+2] * Kx)/w
                    Iv_s[i,j] = np.sum(Cur_img_smooth[i-1:i+2,j-1:j+2] * Ky)/w

            ## CALCULATE TIME GRADIENT AND RADIAL GRADIENT
            It_s = (Cur_img_smooth - Prev_img_smooth)/(self.Time[n] - self.Time[n-1]) # Time gradient
            G_s = U_grid*Iu_s + V_grid*Iv_s # Radial Gradient

            ## SOLVE LEAST SQUARES PROBLEM
            X_s = np.array([
                [f*np.sum(Iu_s**2), f*np.sum(Iu_s*Iv_s), np.sum(G_s*Iu_s)],
                [f*np.sum(Iu_s*Iv_s), f*np.sum(Iv_s**2), np.sum(G_s*Iv_s)],
                [f*np.sum(G_s*Iu_s),  f*np.sum(G_s*Iv_s),  np.sum(G_s**2)]
            ])

            y_s = np.array([
                [-np.sum(Iu_s*It_s)],
                [-np.sum(Iv_s*It_s)],
                [-np.sum(G_s*It_s)]
            ])

            ## SOLVE b VIA PSEUDO-INVERSE
            b_s = np.linalg.pinv(X_s)@y_s
            b_s = b_s.flatten()


            self.OFy_est_s[n] = b_s[0]
            self.OFx_est_s[n] = b_s[1]
            self.Tau_est_s[n] = 1/(b_s[2])

            Prev_img_smooth = Cur_img_smooth

            print(f"Current Image: {n}/{self.Camera_array.shape[0]}")
                
    def Plotter(self): #plot results


        plt.rc('xtick', labelsize=16)    # fontsize of the tick labels
        plt.rc('ytick', labelsize=16)    # fontsize of the tick labels

        fig = plt.figure(1)
        

        ## PLOT TAU VALUES
        ax1 = fig.add_subplot(311)
        ax1.plot(self.Time,self.Tau,color='tab:blue',label='Tau')
        ax1.plot(self.Time,self.Tau_est,color='r',linestyle='--',label='Tau_est')
        ax1.grid()
        ax1.legend(loc='upper right')
        ax1.set_ylabel("Tau [s]")
        ax1.set_xlabel("Time [s]")

        ## PLOT OFx VALUES
        ax2 = fig.add_subplot(312,sharex = ax1)
        ax2.plot(self.Time,self.OFx,color='tab:blue',label='OFx')
        ax2.plot(self.Time,self.OFx_est,color='g',linestyle='--',label='OFx_est')
        ax2.grid()
        ax2.legend(loc='upper right')
        ax2.set_ylabel("OFx [rad/s]")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylim(-10,10)

        ## PLOT OFy VALUES
        ax3 = fig.add_subplot(313,sharex = ax1)
        ax3.plot(self.Time,self.OFy,color='tab:blue',label='OFy')
        ax3.plot(self.Time,self.OFy_est,color='tab:orange',linestyle='--',label='OFy_est')
        ax3.grid()
        ax3.legend(loc='upper right')
        ax3.set_ylabel("OFy [rad/s]")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylim(-10,10)

        
        fig2 = plt.figure(2)

        ## PLOT TAU VALUES
        fig2.suptitle("Smooth Gradient Ceiling", fontsize=26)
        ax1 = fig2.add_subplot(211)
        ax1.plot(self.Time[1:],self.Tau[1:],color = 'tab:blue',label = 'Tau',linewidth=2)
        ax1.plot(self.Time[1:],self.Tau_est[1:],color = 'r',linestyle = '--',label = 'Tau_est', dashes=(5, 4))
        ax1.plot(self.Time[1:],self.Tau_est_s[1:],color = 'g',linestyle = '--',label = 'Tau_est_smoothed', dashes=(5, 3))
        
        ax1.grid()
        ax1.legend(loc='upper right',fontsize=16)
        ax1.set_ylabel("Tau [s]",fontsize=16)
        ax1.set_xlabel("Time [s]",fontsize=16)

        ## PLOT ERROR
        ax2 = fig2.add_subplot(212,sharex = ax1)
        ax2.plot(self.Time[1:],(self.Tau_est[1:] - self.Tau[1:]),color = 'r',label = "Error in Unsmoothed Tau")
        ax2.plot(self.Time[1:],(self.Tau_est_s[1:] - self.Tau[1:]),color = 'g',label = "Error in Smoothed Tau")
        # ax2.hlines(y =  0.05, xmin = Time_arr[-1] - 1, xmax = Time_arr[-1],linestyle = "--", linewidth = 2, color = 'k') #plot desired error bounds
        # ax2.hlines(y = -0.05, xmin = Time_arr[-1] - 1, xmax = Time_arr[-1],linestyle = "--", linewidth = 2, color = 'k')
        # ax2.vlines(x = (Time_arr[-1] - 1), ymin = -0.05, ymax = 0.05, linestyle = "--", linewidth = 2, color = "k")
        # ax2.vlines(x = (Time_arr[-1]), ymin = -0.05, ymax = 0.05, linestyle = "--", linewidth = 2, color = "k")
        ax2.grid()
        ax2.legend(loc='lower right',fontsize=16)
        ax2.set_ylabel("Error",fontsize=16)
        ax2.set_xlabel("Time [s]",fontsize=16)

        plt.show()
        
        # fig, ax = plt.subplots(3,1, sharex = True)
        # ax[0].set_title("TTC Comparison")
        # # ax[0].plot(self.Time,self.TTC_est1,'r', label = 'TTC_est1')
        # ax[0].plot(self.Time,self.Tau,'b',label = 'TTC')
        # ax[0].plot(self.Time,self.TTC_est2,'g',label = 'TTC_est2')
        # ax[0].set_ylabel("TTC (seconds)")
        # ax[0].set_ylim(-1,6)
        # ax[0].grid()
        # ax[0].legend(loc='upper right')

        # ax[1].plot(self.Time,self.Z_pos,'g', label = 'Z position')
        # ax[1].set_ylim(-1,2.5)
        # ax[1].axhline(2.10,color='k',linestyle='dashed',label='Ceiling Height')
        # ax[1].set_ylabel("Position (m)")
        # ax[1].grid()
        # ax[1].legend(loc='lower right')

        # ax[2].plot(self.Time,self.Z_vel,color = 'black', label = 'Z velocity')
        # ax[2].plot(self.Time,self.X_vel,color = 'r', label = 'X velocity')
        # ax[2].set_ylim(-1,4.0)
        # ax[2].set_ylabel("Velocity (m)")
        # ax[2].set_xlabel("Time") 
        # ax[2].grid()
        # ax[2].legend(loc='upper right')


        # #plt.figure(2)
        # fig2 , ax2 = plt.subplots(2,1, sharex = True)
        # ax2[0].set_title('Optical Flow Comparison')
        # ax2[0].plot(self.Time,self.OFy, color = 'black', label = 'OFy')
        # ax2[0].plot(self.Time,self.OFy_est, color = 'r',label = 'OFy_est')
        # ax2[0].set_ylim(-1,10.0)
        # ax2[0].grid()
        # ax2[0].legend()

        # ax2[1].plot(self.Time,self.OFx, color = 'black',label = 'OFx')
        # ax2[1].plot(self.Time,self.OFx_est, color = 'b',label = 'OFx_est')
        # ax2[1].set_xlabel("Time")
        # ax2[1].set_ylabel('Optical Flow')
        # ax2[1].set_ylim(-1,10.0)
        # ax2[1].grid()
        # ax2[1].legend()

        # plt.show()


if __name__ == '__main__':

    DataParser() #init class