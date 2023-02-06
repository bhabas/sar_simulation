import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from tqdm import tqdm,trange
import matplotlib.animation as animation

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(1,BASE_PATH)

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160
# FPS = 60                # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3/2           # Focal Length [m]
O_vp = WIDTH_PIXELS/2   # Pixel X_offset [pixels]
O_up = HEIGHT_PIXELS/2  # Pixel Y_offset [pixels]

FILTER_FLAG = False


class DataParser:

    def __init__(self):

        ## INIT LOGGING PARAMETERS
        # self.FileName = input("Input the name of the log file:\n")
        self.FileName = "FlightLog_Tau_Only_2"
        self.LogDir = f"{BASE_PATH}/crazyflie_projects/Optical_Flow_Estimation/local_logs/{self.FileName}"
        self.FilePath = os.path.join(self.LogDir,self.FileName) + ".csv"
        

        ## LOAD CSV FILE
        self.Data_df = pd.read_csv(self.FilePath,quotechar='"',low_memory=False,comment="#")

        ## LOAD STATE AND CAMERA DATA
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

        ## CONVERT IMAGE DATA FROM STRING TO INTEGER
        self.Image_data = self.Data_df["Camera_Data"].to_numpy()   # Array of camera images
        self.n_imgs = len(self.t)
        Image_list = []
        for n in range(0,self.Image_data.size): 
            Image_list.append(np.fromstring(self.Image_data[n], dtype=int, sep =' ').reshape(WIDTH_PIXELS,HEIGHT_PIXELS))
        self.Image_array = np.asarray(Image_list)

        # CHECK IF DATA HAS ALREADY BEEN COMPILED
        if 'Tau_est' in self.Data_df.columns:
            pre_compiled_Flag = True
            self.Tau_est = self.Data_df['Tau_est'].to_numpy()
            self.Theta_x_est = self.Data_df['Theta_x_est'].to_numpy()
            self.Theta_y_est = self.Data_df['Theta_y_est'].to_numpy()
        else:
            pre_compiled_Flag = False


    def grabImage(self,idx):

        return self.Image_array[idx]

    def grabState(self,state_str,idx=None):

        if idx == None:
            return self.Data_df[state_str].to_numpy()
        else:
            return self.Data_df[state_str].to_numpy()[idx].item()

    def SaveCamera_MP4(self,n=10):
        """
            Saves recorded images from camera into an MP4 file in the log directory
        """        

        U_p,V_p = np.meshgrid(np.arange(0,WIDTH_PIXELS,1),np.arange(0,HEIGHT_PIXELS,1))
        

        fig = plt.figure()
        ax = fig.add_subplot(111)

        t_cur = Parser.grabState(['t'],idx=1)
        t_prev = Parser.grabState(['t'],idx=0)
        t_delta = t_cur - t_prev
        du_dt,dv_dt = self.Calc_OF_LK(self.Image_array[1],self.Image_array[0],t_delta)

        im = ax.imshow(self.Image_array[0], 
                interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',)

        Q = ax.quiver(
            U_p[::n,::n],V_p[::n,::n],
            du_dt[::n,::n],-dv_dt[::n,::n], # Need negative sign for arrow to match correct direction
            color='lime')


        def update(i):

            im.set_data(self.Image_array[i])

            t_cur = Parser.grabState(['t'],idx=i)
            t_prev = Parser.grabState(['t'],idx=i-1)
            t_delta = t_cur - t_prev
            du_dt,dv_dt = self.Calc_OF_LK(self.Image_array[i],self.Image_array[i-1],t_delta)
            Q.set_UVC(-du_dt[::n,::n],-dv_dt[::n,::n])

            print(i)

            return im,Q,

        
        ani = animation.FuncAnimation(fig, update, interval=50, blit=False,frames=range(120,self.n_imgs-1))
        ani.save(f"{self.LogDir}/{self.FileName}.mp4")
        
        # plt.show()

    def Plot_OF_Image(self,cur_img,prev_img,t_delta,n=5):
        """Superimpose optical flow vectors over image

        Args:
            image (np.array): Array of image data
            n (int, optional): Stride between vectors. Defaults to 5.
        """        

        du_dt,dv_dt = self.Calc_OF_LK(cur_img,prev_img,t_delta)
        U_p,V_p = np.meshgrid(np.arange(0,WIDTH_PIXELS,1),np.arange(0,HEIGHT_PIXELS,1))


        fig = plt.figure()
        ax = fig.add_subplot(111)

        ax.imshow(cur_img, interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',)
                
        ax.quiver(
            U_p[::n,::n],V_p[::n,::n],
            du_dt[::n,::n],-dv_dt[::n,::n], # Need negative sign for arrow to match correct direction
            color='g')

        plt.show()

    def Calc_OF_LK(self,cur_img,prev_img,t_delta):

        ## SEPERATED SOBEL KERNAL (U--DIRECTION)
        Ku_1 = np.array([[-1,0,1]]).reshape(3,1)
        Ku_2 = np.array([[ 1,2,1]]).reshape(1,3)


        ## SEPERATED SOBEL KERNAL (V--DIRECTION)
        Kv_1 = np.array([[ 1,2,1]]).reshape(3,1)
        Kv_2 = np.array([[-1,0,1]]).reshape(1,3)


        ## PRE-ALLOCATE INTENSITY GRADIENTS
        I_u = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        I_v = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        I_t = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))

        du_dt = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        dv_dt = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))


        ## CALCULATE IMAGE GRADIENTS
        for v_p in range(1, HEIGHT_PIXELS-1): 
            for u_p in range(1, WIDTH_PIXELS-1):
                I_u[v_p,u_p] = -1/(8*w)*(Ku_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Ku_1)))).item()
                I_v[v_p,u_p] =  1/(8*w)*(Kv_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Kv_1)))).item()
                I_t[v_p,u_p] = (cur_img[v_p,u_p] - prev_img[v_p,u_p])/t_delta   # Time gradient

        # A_temp = np.zeros((2,2))
        # b_temp = np.zeros((2,1))

        # ## ITERATE THROUGH PIXELS AND CALCULATE OPTICAL FLOW
        # for v_p in range(1, HEIGHT_PIXELS-1): 
        #     for u_p in range(1, WIDTH_PIXELS-1):

        #         A_temp[0,0] = np.sum(I_u[v_p-1:v_p+2,u_p-1:u_p+2]*I_u[v_p-1:v_p+2,u_p-1:u_p+2])
        #         A_temp[1,1] = np.sum(I_v[v_p-1:v_p+2,u_p-1:u_p+2]*I_v[v_p-1:v_p+2,u_p-1:u_p+2])
        #         A_temp[1,0] = np.sum(I_u[v_p-1:v_p+2,u_p-1:u_p+2]*I_v[v_p-1:v_p+2,u_p-1:u_p+2])
        #         A_temp[1,0] = A_temp[0,1]


        #         b_temp[0,0] = np.sum(I_t[v_p-1:v_p+2,u_p-1:u_p+2]*I_u[v_p-1:v_p+2,u_p-1:u_p+2])
        #         b_temp[1,0] = np.sum(I_t[v_p-1:v_p+2,u_p-1:u_p+2]*I_v[v_p-1:v_p+2,u_p-1:u_p+2])

        #         vals = np.linalg.solve(A_temp,b_temp)

        #         du_dt[v_p,u_p] = vals[0,0]
        #         dv_dt[v_p,u_p] = vals[1,0]

        # return du_dt,dv_dt

        ## ITERATE THROUGH PIXELS AND CALCULATE OPTICAL FLOW
        for v_p in range(1, HEIGHT_PIXELS-1): 
            for u_p in range(1, WIDTH_PIXELS-1):

                A = np.zeros((9,2))
                b = np.zeros((9,1))
                idx = 0

                
                for ii in range(-1,2):
                    for jj in range(-1,2):
                        A[idx,0] = I_u[v_p+ii,u_p+jj]
                        A[idx,1] = I_v[v_p+ii,u_p+jj]

                        b[idx,0] = -I_t[v_p+ii,u_p+jj]

                        idx += 1

                vals = np.linalg.pinv(A)@b

                du_dt[v_p,u_p] = vals[0,0]
                dv_dt[v_p,u_p] = vals[1,0]

        return du_dt,dv_dt

    


    def Plot_Image(self,image):

        fig = plt.figure()
        ax = fig.add_subplot(111)

        im = ax.imshow(image, interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',animated=True)

        plt.show()


    def Plot_OpticalFlow(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        t = self.grabState('t')
        t_min = min(t)
        t = t - t_min

        Tau = self.grabState('Tau')
        Tau_est = self.grabState('Tau_est')

        ax.plot(t,Tau)
        ax.plot(t,Tau_est)

        ax.grid()

        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Tau [s]')

        ax.set_ylim(0,3)

        plt.show()

    def OF_Calc_Raw(self,cur_img,prev_img,delta_t):
        """Calculate optical flow values without optimization. 
        Derivation in: (Research_Notes_Book_2.pdf)


        Args:
            cur_img (np.array): Array of current image
            prev_img (np.array): Array of previous image
            delta_t (float): Time between images

        Returns:
            np.array: Array of (Theta_x_est,Theta_y_est,Theta_z_est)
        """        

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

    

    def OF_Calc_Opt_Sep(self,cur_img,prev_img,delta_t):

        """Calculate optical flow values with seperable convolution and integer optimizations.
        Derivation in (Research_Notes_Book_2.pdf)

        Args:
            cur_img (np.array): Array of current image
            prev_img (np.array): Array of previous image
            delta_t (float): Time between images

        Returns:
            np.array: Array of (Theta_x_est,Theta_y_est,Theta_z_est)
        """        

        ## SEPERATED SOBEL KERNAL (U--DIRECTION)
        Ku_1 = np.array([[-1,0,1]]).reshape(3,1)
        Ku_2 = np.array([[ 1,2,1]]).reshape(1,3)


        ## SEPERATED SOBEL KERNAL (V--DIRECTION)
        Kv_1 = np.array([[ 1,2,1]]).reshape(3,1)
        Kv_2 = np.array([[-1,0,1]]).reshape(1,3)


        ## PRE-ALLOCATE INTENSITY GRADIENTS
        G_up = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_vp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_rp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))
        G_tp = np.zeros((HEIGHT_PIXELS,WIDTH_PIXELS))


        ## CALCULATE IMAGE GRADIENTS
        for v_p in range(1, HEIGHT_PIXELS-1): 
            for u_p in range(1, WIDTH_PIXELS-1):
                G_up[v_p,u_p] = (Ku_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Ku_1)))).item()
                G_vp[v_p,u_p] = (Kv_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Kv_1)))).item()
                G_rp[v_p,u_p] = (2*(u_p - O_up) + 1)*G_up[v_p,u_p] + (2*(v_p - O_vp) + 1)*G_vp[v_p,u_p]
                G_tp[v_p,u_p] = cur_img[v_p,u_p] - prev_img[v_p,u_p]


        ## SOLVE LEAST SQUARES PROBLEM
        X = np.array([
            [f*np.sum(G_vp*G_vp), -f*np.sum(G_up*G_vp), -w/2*np.sum(G_rp*G_vp)],
            [f*np.sum(G_vp*G_up), -f*np.sum(G_up*G_up), -w/2*np.sum(G_rp*G_up)],
            [f*np.sum(G_vp*G_rp), -f*np.sum(G_up*G_rp), -w/2*np.sum(G_rp*G_rp)]
        ])

        y = np.array([
            [np.sum(G_tp*G_vp)],
            [np.sum(G_tp*G_up)],
            [np.sum(G_tp*G_rp)]
        ])*(8*w/delta_t)

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y
        b = b.flatten()

        return b

    def OpticalFlow_Writer(self,OF_Calc_Func):
        """Calculates optical flow values for all images in log file and appends them to log file
        """        

        Theta_x_est_arr = [np.nan]
        Theta_y_est_arr = [np.nan]
        Tau_est_arr = [np.nan]
        
        with trange(1,self.n_imgs) as n:
            for ii in n:

                ## COLLECT CURRENT AND PREVIOUS IMAGE
                prev_img = self.grabImage(ii-1)
                cur_img = self.grabImage(ii)
                
                ## CALCULATE TIME BETWEEN IMAGES
                t_prev = self.grabState('t',ii-1)
                t_cur = self.grabState('t',ii)
                t_delta = t_cur - t_prev

                ## CALCULATE OPTICAL FLOW VALUES
                Theta_x_est,Theta_y_est,Theta_z_est = OF_Calc_Func(cur_img,prev_img,t_delta)
                Theta_x_est,Theta_y_est,Tau_est = np.round([Theta_x_est,Theta_y_est,1/Theta_z_est],3)

                n.set_description(f"Tau_est: {Tau_est:.3f} \t Theta_x_est: {Theta_x_est:.3f}")
                n.ncols = 120


                ## RECORD ESTIMATED OPTICAL FLOW VALUES
                Theta_x_est_arr.append(Theta_x_est)
                Theta_y_est_arr.append(Theta_y_est)
                Tau_est_arr.append(Tau_est)


        ## APPEND COMPILED DATA TO DATAFRAME
        self.df = self.Data_df.iloc[:,:-1]
        self.df['Tau_est'] = Tau_est_arr
        self.df['Theta_x_est'] = Theta_x_est_arr
        self.df['Theta_y_est'] = Theta_y_est_arr
        self.df['Camera_Data'] = self.Data_df.iloc[:,-1]

        self.df.to_csv(self.FilePath,index=False)



        

if __name__ == '__main__':

    Parser = DataParser() 
    # Parser.OpticalFlow_Writer(Parser.OF_Calc_Opt_Sep)
    Parser.SaveCamera_MP4()
    # Parser.Plot_OpticalFlow()

    # Parser.Plot_Image(Parser.grabImage(150))


    # cur_img = Parser.grabImage(150)
    # prev_img = Parser.grabImage(149)

    # t_cur = Parser.grabState(['t'],idx=150)
    # t_prev = Parser.grabState(['t'],idx=149)
    # t_delta = t_cur - t_prev

    # Parser.Plot_OF_Image(Parser.grabImage(150),Parser.grabImage(149),t_delta)

