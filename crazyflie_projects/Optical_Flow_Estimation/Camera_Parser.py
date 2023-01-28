import numpy as np
import pandas as pd
import getpass
import os
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import cv2 as cv


## CAMERA PARAMETERS
WIDTH_PIXELS = 4
HEIGHT_PIXELS = 4
FPS = 60               # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3/2          # Focal Length [m]
O_vp = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_up = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]

FILTER_FLAG = False


class DataParser:

    def __init__(self):

        ## INIT LOGGING PARAMETERS
        self.Username = getpass.getuser()
        self.DirPath = f'/home/{self.Username}/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Optical_Flow_Estimation/local_logs' 
        # self.FileName = input("Input the name of the log file:\n")
        self.FileName = "FlightLog_Tau_Only.csv"
        self.FilePath = os.path.join(self.DirPath,self.FileName)

        if self.FileName.find("Compiled") == -1:
            pre_compiled_Flag = False
        else:
            pre_compiled_Flag = True


        ## PARSE CSV DATA
        self.Data_df = pd.read_csv(self.FilePath,quotechar='"',low_memory=False,comment="#")
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


        # CHECK IF DATA HAS ALREADY BEEN COMPILED
        if pre_compiled_Flag == True:
            self.Tau_est = self.Data_df['Tau_est'].to_numpy()
            self.Theta_x_est = self.Data_df['Theta_x_est'].to_numpy()
            self.Theta_y_est = self.Data_df['Theta_y_est'].to_numpy()


        # ## CONVERT IMAGE DATA FROM STRING TO INTEGER
        # self.Camera_array = np.zeros((len(self.Camera_data), WIDTH_PIXELS*HEIGHT_PIXELS)) # Allocate space for camera data
        # for n in range(0,self.Camera_data.size): # Make each row an array of ints
        #     self.Camera_array[n] = np.fromstring(self.Camera_data[n], dtype=int, sep =' ')

    def grabImage(self,idx):

        return self.Camera_array[idx].reshape(WIDTH_PIXELS,HEIGHT_PIXELS)

    def grabState(self,state_str,idx=None):

        if idx == None:
            return self.Data_df[state_str].to_numpy()
        else:
            return self.Data_df[state_str][idx]


    def Plot_Image(self,image_array):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        ax.imshow(image_array.reshape(WIDTH_PIXELS,HEIGHT_PIXELS), interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',)

        plt.show()

    def OF_Calc_Raw(self,cur_img,prev_img,delta_t):

        ## DEFINE KERNALS USED TO CALCULATE INTENSITY GRADIENTS
        Kv = np.array([ # SOBEL KERNAL (U-DIRECTION)
            [-1,-2,-1],
            [ 0, 0, 0],
            [ 1, 2, 1]
        ]) 

        Ku = np.array([ # SOBEL KERNAL (V--DIRECTION)
            [ -1, 0, 1],
            [ -2, 0, 2],
            [ -1, 0, 1]
        ])

        
        ## PRE-ALLOCATE IMAGE ARRAY [pixels]
        u_p = np.arange(0,WIDTH_PIXELS,1)
        v_p = np.arange(0,HEIGHT_PIXELS,1)
        U_p,V_p = np.meshgrid(u_p,v_p)

        ## DEFINE IMAGE SENSOR COORDS [m]
        U_grid = -((U_p - O_up)*w + w/2)
        V_grid =  ((V_p - O_vp)*w + w/2)

        ## PRE-ALLOCATE INTENSITY GRADIENTS
        Iv = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        Iu = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))


        ## CALCULATE IMAGE GRADIENTS
        for ii in range(1, HEIGHT_PIXELS-1): 
            for jj in range(1, WIDTH_PIXELS-1):
                Iu[ii,jj] = -1/(8*w)*np.sum(cur_img[ii-1:ii+2,jj-1:jj+2] * Ku)
                Iv[ii,jj] =  1/(8*w)*np.sum(cur_img[ii-1:ii+2,jj-1:jj+2] * Kv)

        ## CALCULATE TIME GRADIENT AND RADIAL GRADIENT
        It = (cur_img - prev_img)/delta_t   # Time gradient
        Ir = U_grid*Iu + V_grid*Iv          # Radial Gradient


        ## SOLVE LEAST SQUARES PROBLEM
        X = np.array([
            [f*np.sum(Iv*Iv), f*np.sum(Iu*Iv), -np.sum(Ir*Iv)],
            [f*np.sum(Iv*Iu), f*np.sum(Iu*Iu), -np.sum(Ir*Iu)],
            [f*np.sum(Iv*Ir), f*np.sum(Iu*Ir), -np.sum(Ir*Ir)]
        ])

        y = np.array([
            [np.sum(It*Iv)],
            [np.sum(It*Iu)],
            [np.sum(It*Ir )]
        ])

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y
        b = b.flatten()

        return b

    def OF_Calc_Opt(self,cur_img,prev_img,delta_t):

        ## DEFINE KERNALS USED TO CALCULATE INTENSITY GRADIENTS
        Kv = np.array([ # SOBEL KERNAL (U-DIRECTION)
            [-1,-2,-1],
            [ 0, 0, 0],
            [ 1, 2, 1]
        ]) 

        Ku = np.array([ # SOBEL KERNAL (V--DIRECTION)
            [ -1, 0, 1],
            [ -2, 0, 2],
            [ -1, 0, 1]
        ])

        
        ## PRE-ALLOCATE IMAGE ARRAY [pixels]
        u_p = np.arange(0,WIDTH_PIXELS,1)
        v_p = np.arange(0,HEIGHT_PIXELS,1)
        U_p,V_p = np.meshgrid(u_p,v_p)

        ## PRE-ALLOCATE INTENSITY GRADIENTS
        G_up = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_vp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_rp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))



        Iv = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        Iu = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        Ir = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))



        ## CALCULATE IMAGE GRADIENTS
        for v_p in range(1, HEIGHT_PIXELS-1): 
            for u_p in range(1, WIDTH_PIXELS-1):
                G_up[v_p,u_p] = np.sum(cur_img[v_p-1:v_p+2,u_p-1:u_p+2] * Ku)
                G_vp[v_p,u_p] = np.sum(cur_img[v_p-1:v_p+2,u_p-1:u_p+2] * Kv)
                G_rp[v_p,u_p] = (u_p - O_up + 1/2)*G_up[v_p,u_p] + (v_p - O_vp + 1/2)*G_vp[v_p,u_p]

        ## CALCULATE TIME GRADIENT AND RADIAL GRADIENT
        It = (cur_img - prev_img)/delta_t   # Time gradient


        ## SOLVE LEAST SQUARES PROBLEM
        X = 1/(8*w)*np.array([
            [  f*np.sum(G_vp*G_vp),   -f*np.sum(G_up*G_vp),   -w*np.sum(G_rp*G_vp)],
            [  f*np.sum(G_vp*G_up),   -f*np.sum(G_up*G_up),    w*np.sum(G_rp*G_up)],
            [w*f*np.sum(G_vp*G_rp), -w*f*np.sum(G_up*G_rp), -w*w*np.sum(G_rp*G_rp)]
        ])

        y = np.array([
            [  np.sum(It*G_vp)],
            [  np.sum(It*G_up)],
            [w*np.sum(It*G_rp)]
        ])

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y
        b = b.flatten()

        return b

    def OpticalFlow_Writer(self):

        Theta_x_arr = [np.nan]
        Theta_y_arr = [np.nan]
        Tau_arr = [np.nan]

        for ii in range(1,len(self.t)):
            
            ## COLLECT CURRENT AND PREVIOUS IMAGE
            prev_img = self.grabImage(ii-1)
            cur_img = self.grabImage(ii)
            
            ## CALCULATE TIME BETWEEN IMAGES
            t_prev = self.grabState('t',ii-1)
            t_cur = self.grabState('t',ii)
            t_delta = t_cur - t_prev

            ## CALCULATE OPTICAL FLOW VALUES
            Theta_x_est,Theta_y_est,Theta_z_est = self.OF_Calc_Opt(cur_img,prev_img,t_delta)
            print(Theta_z_est**-1)

            Theta_x_est,Theta_y_est,Tau_est = np.round([Theta_x_est,Theta_y_est,1/Theta_z_est],3)

            ## RECORD ESTIMATED OPTICAL FLOW VALUES
            Theta_x_arr.append(Theta_x_est)
            Theta_y_arr.append(Theta_y_est)
            Tau_arr.append(Tau_est)

            print(f"Compiling Data: {ii}/{len(self.t)}")


        ## CREATE NEW DATAFRAME
        self.df = self.Data_df.iloc[:,:-1]
        self.df['Tau_est'] = Tau_arr
        self.df['Theta_x_est'] = Theta_x_arr
        self.df['Theta_y_est'] = Theta_y_arr
        self.df['Camera_Data'] = self.Data_df.iloc[:,-1]

        FilePath = os.path.join(self.DirPath,f"Compiled_{self.FileName}")
        # self.df.to_csv(FilePath,index=False)



        

if __name__ == '__main__':

    Parser = DataParser() 
    # Parser.OpticalFlow_Writer()

    t_prev = Parser.grabState('t',0)
    t_cur = Parser.grabState('t',1)
    t_delta = t_cur - t_prev

    # img_prev = Parser.grabImage(0)
    # img_cur = Parser.grabImage(1)

    img_cur = np.array([
        [1,1,0,0],
        [1,1,0,0],
        [1,1,0,0],
        [1,1,0,0]
        ])

    img_prev = np.array([
        [0,0,1,1],
        [0,0,1,1],
        [0,0,1,1],
        [0,0,1,1]
        ])


    print(Parser.OF_Calc_Raw(img_cur,img_prev,t_delta))
    print(Parser.OF_Calc_Opt(img_cur,img_prev,t_delta))
