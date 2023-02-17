import numpy as np
import pandas as pd
import os
import yaml
from tqdm import trange
from scipy.ndimage import convolve


import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(1,BASE_PATH)        
np.set_printoptions(threshold = sys.maxsize) # Print full string without truncation


class DataParser:

    def __init__(self,FileName):

        ## INIT LOGGING PARAMETERS
        self.FileName = FileName
        self.LogDir = f"{BASE_PATH}/crazyflie_projects/Optical_Flow_Estimation/local_logs/"
        self.FileDir = os.path.join(self.LogDir,self.FileName)   
        self.FilePath = os.path.join(self.FileDir,self.FileName + ".csv")    
        self.ConfigPath = os.path.join(self.FileDir,"Config.yaml")

        
        self.Load_Config_File()
        self.LoadData()
        self.Write_Config_File()

        
    
    def LoadData(self):

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
            Image_list.append(np.fromstring(self.Image_data[n], dtype=int, sep =' ').reshape(self.N_up,self.N_vp))
        self.Image_array = np.asarray(Image_list)

        # CHECK IF DATA HAS ALREADY BEEN COMPILED
        if 'Tau_est' in self.Data_df.columns:
            self.Tau_est = self.Data_df['Tau_est'].to_numpy()
            self.Theta_x_est = self.Data_df['Theta_x_est'].to_numpy()
            self.Theta_y_est = self.Data_df['Theta_y_est'].to_numpy()

    def Load_Config_File(self):

        with open(self.ConfigPath, 'r') as file:
            self.YAML_data = yaml.safe_load(file)

        self.N_up = self.YAML_data['IMAGE_SETTINGS']['N_up']    # IMAGE WIDTH IN PIXELS
        self.N_vp = self.YAML_data['IMAGE_SETTINGS']['N_vp']    # IMAGE HEIGHT IN PIXELS

        self.f = self.YAML_data['IMAGE_SETTINGS']['f']          # Focal Length [m]
        self.FOV = self.YAML_data['IMAGE_SETTINGS']['FOV']      # Field of View [deg]
        self.FOV_rad = np.radians(self.FOV)                 # Field of View [rad]
        self.IW = self.YAML_data['IMAGE_SETTINGS']['IW']        # Image Sensor Width [m]

        
    def Write_Config_File(self):

        self.YAML_data['IMAGE_SETTINGS']['N_up'] = self.N_up
        self.YAML_data['IMAGE_SETTINGS']['N_vp'] = self.N_vp

        with open(self.ConfigPath, 'w') as outfile:
            yaml.dump(self.YAML_data,outfile,default_flow_style=False,sort_keys=False) 
    

    def grabImage(self,idx):
        """Return image array for a the given index position from logfile

        Args:
            idx (int): Index location of image

        Returns:
            np.array: Array of image brightness values
        """        

        return self.Image_array[idx]

    def grabState(self,state_str,idx=None):
        """Returns array of values provided in log file

        Args:
            state_str (str): String matching state name in log file
            idx (int, optional): Provide index if single value is needed. Defaults to None.

        Returns:
            np.array: Return array of values or single value if index is provided
        """        

        if idx == None:
            return self.Data_df[state_str].to_numpy()
        else:
            return self.Data_df[state_str].to_numpy()[idx].item()

    def DataOverview(self,n=10,frame_idx=1,save_fig=False):

        ## GENERATE FIGURE
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(3, 2,width_ratios=[0.4,0.6])

        ## CALCULATE INITIAL OPTICAL FLOW VALUES
        t_cur = Parser.grabState(['t'],idx=frame_idx)
        t_prev = Parser.grabState(['t'],idx=frame_idx-1)
        t_delta = t_cur - t_prev

        prev_img = self.Image_array[frame_idx-1]
        cur_img = self.Image_array[frame_idx]


        N_vp = cur_img.shape[0]
        N_up = cur_img.shape[1]
        U_p,V_p = np.meshgrid(np.arange(0,N_up,1),np.arange(0,N_vp,1))

        n = int(N_up/n)
        du_dt,dv_dt = self.Calc_OF_LK(cur_img,prev_img,t_delta,n)


        ## GENERATE IMAGE AXES
        Img_ax = fig.add_subplot(gs[:,0])
        Img_ax.set_title(fr"$V_\perp$: {self.vx[frame_idx]:.2f} [m/s] | $V_\parallel$: {-self.vz[frame_idx]:.2f} | $D_\perp$: {self.D_perp[frame_idx]:.2f} [m]"
                             f"\n"
                             f"Frame: {frame_idx:03d} | FPS: {int(1/t_delta):03d} [Hz]")
        # IMAGE PLOT
        im = Img_ax.imshow(cur_img, 
                interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',)

        # OPTICAL FLOW QUIVER PLOT 
        Q = Img_ax.quiver(
            U_p[1::n,1::n],V_p[1::n,1::n],
            -du_dt[1::n,1::n],-dv_dt[1::n,1::n], # Need negative sign for arrows to match correct direction
            color='lime')


        ## GENERATE TAU_EST PLOT
        Tau_ax = fig.add_subplot(gs[0,1])
        Tau_ax.plot(self.t[1:]-self.t.min(), self.Tau_est[1:])
        Tau_ax.plot(self.t[1:]-self.t.min(),self.Tau[1:])
        Tau_marker, = Tau_ax.plot(self.t[frame_idx]-self.t.min(), self.Tau_est[frame_idx],marker='o')

        Tau_ax.set_ylim(0,1)
        Tau_ax.set_ylabel("Tau [s]")
        Tau_ax.set_title(fr"Tau Error | (Bias: {np.nanmean(np.abs(self.Tau[1:] - self.Tau_est[1:])):.2f} $\pm 2\sigma$: {2*np.nanstd(self.Tau_est[1:]):.2f})")
        Tau_ax.tick_params('x', labelbottom=False)
        Tau_ax.grid()


        ## GENERATE THETA_X_EST PLOT
        Theta_x_ax = fig.add_subplot(gs[1,1])
        Theta_x_ax.plot(self.t[1:]-self.t.min(), self.Theta_x_est[1:])
        Theta_x_ax.plot(self.t[1:]-self.t.min(),self.Theta_x[1:])
        Theta_x_marker, = Theta_x_ax.plot(self.t[frame_idx]-self.t.min(), self.Theta_x_est[frame_idx],marker='o')

        Theta_x_ax.set_ylim(-2,10)
        Theta_x_ax.set_ylabel("Theta_x [1/s]")
        Theta_x_ax.set_title(fr"Theta_x Error | (Bias: {np.nanmean(np.abs(self.Theta_x[1:] - self.Theta_x_est[1:])):.2f}  $\pm 2\sigma$: {2*np.nanstd(self.Theta_x_est[1:]):.2f})")
        Theta_x_ax.tick_params('x', labelbottom=False)
        Theta_x_ax.grid()


        ## GENERATE THETA_Y_EST PLOT
        Theta_y_ax = fig.add_subplot(gs[2,1])
        Theta_y_ax.plot(self.t[1:]-self.t.min(), self.Theta_y_est[1:])
        Theta_y_ax.plot(self.t[1:]-self.t.min(),self.Theta_y[1:])
        Theta_y_marker, = Theta_y_ax.plot(self.t[frame_idx]-self.t.min(), self.Theta_y_est[frame_idx],marker='o')

        Theta_y_ax.set_ylim(-2,10)
        Theta_y_ax.set_ylabel("Theta_y [1/s]")
        Theta_y_ax.set_title(fr"Theta_y Error | (Bias: {np.nanmean(np.abs(self.Theta_y[1:] - self.Theta_y_est[1:])):.2f}  $\pm 2\sigma$: {2*np.nanstd(self.Theta_y_est[1:]):.2f})")
        Theta_y_ax.grid()

        fig.tight_layout()

        if save_fig == True:
            plt.savefig(f"{self.FileDir}/Data_Plot--Frame_{frame_idx:03d}.png")
        plt.show()


        
      
        # ani.save(f"{self.FileDir}/{self.FileName}.mp4")

    def DataOverview_MP4(self,n=10,frame_limit=None):
        """
            Saves recorded images from camera into an MP4 file in the log directory
        """        

        if frame_limit == None:
            frame_limit = self.n_imgs-1

        ## GENERATE FIGURE
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(3, 2,width_ratios=[0.4,0.6])

        ## CALCULATE INITIAL OPTICAL FLOW VALUES
        t_cur = Parser.grabState(['t'],idx=1)
        t_prev = Parser.grabState(['t'],idx=0)
        t_delta = t_cur - t_prev

        prev_img = self.Image_array[0]
        cur_img = self.Image_array[1]

        
        N_vp = cur_img.shape[0]
        N_up = cur_img.shape[1]
        U_p,V_p = np.meshgrid(np.arange(0,N_up,1),np.arange(0,N_vp,1))

        n = int(N_up/n)
        du_dt,dv_dt = self.Calc_OF_LK(cur_img,prev_img,t_delta,n)

        ## GENERATE IMAGE AXES
        Img_ax = fig.add_subplot(gs[:,0])
        Img_ax.set_title(fr"$V_\perp$: {self.vx[0]:.2f} [m/s] | $V_\parallel$: {-self.vz[0]:.2f} | $D_\perp$: {self.D_perp[0]:.2f} [m]"
                             f"\n"
                             f"Frame: {0:03d} | FPS: {int(1/t_delta):03d} [Hz]")
        # IMAGE PLOT
        im = Img_ax.imshow(cur_img, 
                interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',)

        # OPTICAL FLOW QUIVER PLOT 
        Q = Img_ax.quiver(
            U_p[1::n,1::n],V_p[1::n,1::n],
            -du_dt[1::n,1::n],-dv_dt[1::n,1::n], # Need negative sign for arrows to match correct direction
            color='lime')


        ## GENERATE TAU_EST PLOT
        Tau_ax = fig.add_subplot(gs[0,1])
        Tau_ax.plot(self.t[1:]-self.t.min(), self.Tau_est[1:])
        Tau_ax.plot(self.t[1:]-self.t.min(),self.Tau[1:])
        Tau_marker, = Tau_ax.plot([],[],marker='o')

        Tau_ax.set_ylim(0,1)
        Tau_ax.set_ylabel("Tau [s]")
        Tau_ax.set_title(fr"Tau Error | (Bias: {np.nanmean(np.abs(self.Tau[1:] - self.Tau_est[1:])):.2f} $\pm 2\sigma$: {2*np.nanstd(self.Tau_est[1:]):.2f})")
        Tau_ax.tick_params('x', labelbottom=False)
        Tau_ax.grid()


        ## GENERATE THETA_X_EST PLOT
        Theta_x_ax = fig.add_subplot(gs[1,1])
        Theta_x_ax.plot(self.t[1:]-self.t.min(), self.Theta_x_est[1:])
        Theta_x_ax.plot(self.t[1:]-self.t.min(),self.Theta_x[1:])
        Theta_x_marker, = Theta_x_ax.plot([],[],marker='o')

        Theta_x_ax.set_ylim(-2,10)
        Theta_x_ax.set_ylabel("Theta_x [1/s]")
        Theta_x_ax.set_title(fr"Theta_x Error | (Bias: {np.nanmean(np.abs(self.Theta_x[1:] - self.Theta_x_est[1:])):.2f}  $\pm 2\sigma$: {2*np.nanstd(self.Theta_x_est[1:]):.2f})")
        Theta_x_ax.tick_params('x', labelbottom=False)
        Theta_x_ax.grid()


        ## GENERATE THETA_Y_EST PLOT
        Theta_y_ax = fig.add_subplot(gs[2,1])
        Theta_y_ax.plot(self.t[1:]-self.t.min(), self.Theta_y_est[1:])
        Theta_y_ax.plot(self.t[1:]-self.t.min(),self.Theta_y[1:])
        Theta_y_plot, = Theta_y_ax.plot([],[],marker='o')

        Theta_y_ax.set_ylim(-2,10)
        Theta_y_ax.set_ylabel("Theta_y [1/s]")
        Theta_y_ax.set_title(fr"Theta_y Error | (Bias: {np.nanmean(np.abs(self.Theta_y[1:] - self.Theta_y_est[1:])):.2f}  $\pm 2\sigma$: {2*np.nanstd(self.Theta_y_est[1:]):.2f})")
        Theta_y_ax.grid()

        fig.tight_layout()

        def update(i):
            
            ## UPDATE OPTICAL FLOW
            t_cur = Parser.grabState(['t'],idx=i)
            t_prev = Parser.grabState(['t'],idx=i-1)
            t_delta = t_cur - t_prev

            prev_img = self.Image_array[i-1]
            cur_img = self.Image_array[i]
            du_dt,dv_dt = self.Calc_OF_LK(cur_img,prev_img,t_delta,n)
            Q.set_UVC(-du_dt[1::n,1::n],-dv_dt[1::n,1::n])
            
            ## UPDATE IMAGE
            Img_ax.set_title(fr"$V_\perp$: {self.vx[i]:.2f} [m/s] | $V_\parallel$: {-self.vz[i]:.2f} | $D_\perp$: {self.D_perp[i]:.2f} [m]"
                             f"\n"
                             f"Frame: {i:03d} | FPS: {int(1/t_delta):03d} [Hz]")
            im.set_data(cur_img)
            
            ##  UPDATE MARKER LOCATIONS
            Tau_marker.set_data(self.t[i]-self.t.min(),self.Tau_est[i])
            Theta_x_marker.set_data(self.t[i]-self.t.min(),self.Theta_x_est[i])
            Theta_y_plot.set_data(self.t[i]-self.t.min(),self.Theta_y_est[i])

            ## PRINT PROGRESS
            print(f"Image: {i:03d}/{frame_limit:03d}")

            return im,Q,Tau_marker,Theta_x_marker,Theta_y_plot

        
        ani = animation.FuncAnimation(fig, update, interval=100, blit=False,frames=range(2,frame_limit))
        ani.save(f"{self.FileDir}/{self.FileName}.mp4")

    def OpticalFlow_MP4(self,n=10,frame_limit=None):
        """
            Saves recorded images from camera into an MP4 file in the log directory
        """        
        if frame_limit == None:
            frame_limit = self.n_imgs-1

        ## GENERATE FIGURE
        fig = plt.figure(figsize=(6,6))

        ## CALCULATE INITIAL OPTICAL FLOW VALUES
        t_cur = Parser.grabState(['t'],idx=1)
        t_prev = Parser.grabState(['t'],idx=0)
        t_delta = t_cur - t_prev

        prev_img = self.Image_array[0]
        cur_img = self.Image_array[1]
        du_dt,dv_dt = self.Calc_OF_LK(cur_img,prev_img,t_delta)
        
        N_vp = cur_img.shape[0]
        N_up = cur_img.shape[1]
        U_p,V_p = np.meshgrid(np.arange(0,N_up,1),np.arange(0,N_vp,1))

        ## GENERATE IMAGE AXES
        Img_ax = fig.add_subplot(111)
        Img_ax.set_title(fr"$V_\perp$: {self.vx[0]:.2f} [m/s] | $V_\parallel$: {self.vy[0]:.2f} | $D_\perp$: {self.D_perp[0]:.2f} [m]"
                             f"\n"
                             f"Frame: {0:03d} | FPS: {int(1/t_delta):03d} [Hz]")
        # IMAGE PLOT
        im = Img_ax.imshow(self.Image_array[0], 
                interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',)

        # OPTICAL FLOW QUIVER PLOT 
        Q = Img_ax.quiver(
            U_p[1::n,1::n],V_p[1::n,1::n],
            -du_dt[1::n,1::n],-dv_dt[1::n,1::n], # Need negative sign for arrows to match correct direction
            color='lime')

        def update(i):
            
            ## UPDATE OPTICAL FLOW
            t_cur = Parser.grabState(['t'],idx=i)
            t_prev = Parser.grabState(['t'],idx=i-1)
            t_delta = t_cur - t_prev
            du_dt,dv_dt = self.Calc_OF_LK(self.Image_array[i],self.Image_array[i-1],t_delta,n=10)
            Q.set_UVC(-du_dt[1::n,1::n],-dv_dt[1::n,1::n])
            
            ## UPDATE IMAGE
            Img_ax.set_title(fr"$V_\perp$: {self.vx[i]:.2f} [m/s] | $V_\parallel$: {self.vy[i]:.2f} | $D_\perp$: {self.D_perp[i]:.2f} [m]"
                             f"\n"
                             f"Frame: {i:03d} | FPS: {int(1/t_delta):03d} [Hz]")
            im.set_data(self.Image_array[i])
           
            ## PRINT PROGRESS
            print(f"Image: {i:03d}/{frame_limit:03d}")

            return im,Q,
        
        ani = animation.FuncAnimation(fig, update, interval=100, blit=False,frames=range(2,frame_limit))
        ani.save(f"{self.LogDir}/L-K_{self.FileName}.mp4")
        
    def Plot_OF_Image(self,cur_img,prev_img,t_delta,n=10):
        """Superimpose optical flow vectors over an image

        Args:
            Args:
            cur_img (np.array): Numpy array of the current image
            prev_img (np.array): Numpy array of the previous image
            t_delta (float): Time between images
            n (int, optional): Stride to calculate optical flow vectors over. Defaults to 10.
        """        
        N_vp = cur_img.shape[0]
        N_up = cur_img.shape[1]
        w = self.IW/N_up

        ## CALCULATE OPTICAL FLOW VECTORS VIA LUCAS-KANADE ALGORITHM
        du_dt,dv_dt = self.Calc_OF_LK(cur_img,prev_img,t_delta,n)
        U_p,V_p = np.meshgrid(np.arange(0,N_up,1),np.arange(0,N_vp,1))


        ## GENERATE FIGURE
        fig = plt.figure()
        ax = fig.add_subplot(111)

        ## PLOT IMAGE
        ax.imshow(cur_img, interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',)
        
        ## PLOT OPTICAL FLOW VECTORS
        ax.quiver(
            U_p[1::n,1::n],V_p[1::n,1::n],
            -du_dt[1::n,1::n],-dv_dt[1::n,1::n], # Need negative sign for arrow to match correct direction
            color='lime')

        plt.show()

    def Calc_OF_LK(self,cur_img,prev_img,t_delta,n=1):
        """Calculates series of optical flow vectors between images via Lucas-Kanade algorithm.

        Args:
            cur_img (np.array): Numpy array of the current image
            prev_img (np.array): Numpy array of the previous image
            t_delta (float): Time between images
            n (int, optional): Stride to calculate optical flow vectors over. Defaults to 10.

        Returns:
            (du_dt,dv_dt): A set of numpy arrays matching original image size and 
            containing the optical flow values in each direction
        """        

        N_vp = cur_img.shape[0]
        N_up = cur_img.shape[1]
        w = self.IW/N_up

        ## SEPERATED SOBEL KERNAL (U--DIRECTION)
        Ku_1 = np.array([[-1,0,1]]).reshape(3,1)
        Ku_2 = np.array([[ 1,2,1]]).reshape(1,3)


        ## SEPERATED SOBEL KERNAL (V--DIRECTION)
        Kv_1 = np.array([[ 1,2,1]]).reshape(3,1)
        Kv_2 = np.array([[-1,0,1]]).reshape(1,3)


        ## PRE-ALLOCATE INTENSITY GRADIENTS
        I_u = np.zeros((N_vp,N_up))
        I_v = np.zeros((N_vp,N_up))
        I_t = np.zeros((N_vp,N_up))

        du_dt = np.zeros((N_vp,N_up))
        dv_dt = np.zeros((N_vp,N_up))


        ## CALCULATE IMAGE GRADIENTS
        for v_p in range(1, N_vp-1): 
            for u_p in range(1, N_up-1):
                I_u[v_p,u_p] = -1/(8*w)*(Ku_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Ku_1)))).item()
                I_v[v_p,u_p] =  1/(8*w)*(Kv_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Kv_1)))).item()
                
        I_t = (cur_img - prev_img)/t_delta

        A = np.zeros((2,2))
        b = np.zeros((2,1))

        ## ITERATE THROUGH PIXELS AND CALCULATE OPTICAL FLOW
        for v_p in range(1, N_vp-1,n): 
            for u_p in range(1, N_up-1,n):

                I_u_vec = I_u[v_p-1:v_p+2,u_p-1:u_p+2].flatten()
                I_v_vec = I_v[v_p-1:v_p+2,u_p-1:u_p+2].flatten()
                I_t_vec = I_t[v_p-1:v_p+2,u_p-1:u_p+2].flatten()

                A[0,0] = np.dot(I_u_vec,I_u_vec)
                A[1,0] = np.dot(I_u_vec,I_v_vec)
                A[0,1] = A[1,0]
                A[1,1] = np.dot(I_v_vec,I_v_vec)

                b[0,0] = -np.dot(I_t_vec,I_u_vec)
                b[1,0] = -np.dot(I_t_vec,I_v_vec)

                vals = np.linalg.pinv(A).dot(b)
                du_dt[v_p,u_p],dv_dt[v_p,u_p] = vals.flatten()

        return du_dt,dv_dt

    def Plot_Image(self,image):
        """Shows an image plot given by an array of brightness values

        Args:
            image (np.array): Array of image brightness values
        """     

        fig = plt.figure()
        ax = fig.add_subplot(111)

        ax.imshow(image, interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',animated=True)

        plt.show()

    def Generate_Pattern(self,Surf_width=2,Surf_Height=2,L=0.5,pixel_density=200,save_img=False,X_cam=0.0,D_cam=0.5):
        """Save show pattern and save image to file. As well, display camera boundaries for validation

        Args:
            Surf_width (int, optional): Width of pattern to preserve pixel density. Defaults to 2.
            Surf_Height (int, optional): Heigt of pattern to preserve pixel density. Defaults to 2.
            L (float, optional): Feature width of sin wave. Defaults to 0.5.
            pixel_density (int, optional): Number of pixels per meter. Defaults to 100.
            save_img (bool, optional): Save image to file. Defaults to False.
            X_cam (float, optional): X location of camera. Defaults to 0.5.
            D_cam (float, optional): Distance of camera from pattern surface. Defaults to 0.5.
        """        

        ## GENERATE PATTERN BOUNDS
        x = np.linspace(-0.5*Surf_width, 0.5*Surf_width, pixel_density*Surf_width)
        y = np.linspace(-0.5*Surf_Height,0.5*Surf_Height,pixel_density*Surf_Height)
        X,Y = np.meshgrid(x,y)

        I_0 = 255
        L *=2

        ## GENERATE PATTERN
        I = I_0/2*np.cos(2*np.pi/L*X)*np.cos(2*np.pi/L*Y) + 255/2
        # I = np.where(I < 128,0,255)

        ## GENERATE CAMERA BOUNDS
        Img_Width = 2*np.tan(self.FOV_rad/2)*D_cam
        Img_Height = 2*np.tan(self.FOV_rad/2)*D_cam

        
        ## CREATE FIGURE OF PATTERN
        fig = plt.figure()
        ax = fig.add_subplot(111)

        ## PLOT PATTERN AND CAMERA BOUNDARY
        ax.imshow(I, interpolation='none', 
            vmin=0, vmax=255, cmap=cm.gray,
            origin='upper',
            extent=[x.min(),x.max(),y.min(),y.max()]
            )

        ax.add_patch(
            Rectangle(
                (X_cam-Img_Width/2,-Img_Height/2),
                Img_Width,
                Img_Height,
                lw=1,fill=False,color="tab:blue")
            )

        ## SHOW PLOT
        plt.show()

        ## SAVE PATTERN TO FILE
        if save_img == True:
            plt.imsave(
                f'{BASE_PATH}/crazyflie_projects/Optical_Flow_Estimation/Surface_Patterns/Strip_Pattern_W_{Surf_width}-H_{Surf_Height}-L_{L/2}.png', 
                I, 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',
            )

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

        N_vp = cur_img.shape[0]
        N_up = cur_img.shape[1]
        w = self.IW/N_up


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
        u_p = np.arange(0,N_up,1)
        v_p = np.arange(0,N_vp,1)
        U_p,V_p = np.meshgrid(u_p,v_p)

        ## DEFINE IMAGE SENSOR COORDS [m]
        U_grid = -((U_p - N_up/2)*w + w/2)
        V_grid =  ((V_p - N_vp/2)*w + w/2)

        ## PRE-ALLOCATE INTENSITY GRADIENTS
        Iv = np.zeros((N_vp,N_up))
        Iu = np.zeros((N_vp,N_up))


        ## CALCULATE IMAGE GRADIENTS
        for ii in range(1, N_vp-1): 
            for jj in range(1, N_up-1):
                Iu[ii,jj] = -1/(8*w)*np.sum(cur_img[ii-1:ii+2,jj-1:jj+2] * Ku)
                Iv[ii,jj] =  1/(8*w)*np.sum(cur_img[ii-1:ii+2,jj-1:jj+2] * Kv)

        ## CALCULATE TIME GRADIENT AND RADIAL GRADIENT
        It = (cur_img - prev_img)/delta_t   # Time gradient
        Ir = U_grid*Iu + V_grid*Iv          # Radial Gradient


        ## SOLVE LEAST SQUARES PROBLEM
        X = np.array([
            [self.f*np.sum(Iv*Iv), self.f*np.sum(Iu*Iv), -np.sum(Ir*Iv)],
            [self.f*np.sum(Iv*Iu), self.f*np.sum(Iu*Iu), -np.sum(Ir*Iu)],
            [self.f*np.sum(Iv*Ir), self.f*np.sum(Iu*Ir), -np.sum(Ir*Ir)]
        ])

        y = np.array([
            [np.sum(It*Iv)],
            [np.sum(It*Iu)],
            [np.sum(It*Ir )]
        ])

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y

        return b.flatten()

    def OF_Calc_PyOpt(self,cur_img,prev_img,delta_t):
        """Calculate optical flow values with python optimization. 
        Derivation in: (Research_Notes_Book_2.pdf)

        Args:
            cur_img (np.array): Array of current image
            prev_img (np.array): Array of previous image
            delta_t (float): Time between images

        Returns:
            np.array: Array of (Theta_x_est,Theta_y_est,Theta_z_est)
        """      

        N_vp = cur_img.shape[0]
        N_up = cur_img.shape[1]
        w = self.IW/N_up

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
        u_p = np.arange(0,N_up,1)
        v_p = np.arange(0,N_vp,1)
        
        ## PRE-ALLOCATE INTENSITY GRADIENTS
        G_up = np.zeros((N_vp,N_up))
        G_vp = np.zeros((N_vp,N_up))
        G_rp = np.zeros((N_vp,N_up))
        G_tp = np.zeros((N_vp,N_up))


        ## CALCULATE IMAGE GRADIENTS VIA PYTHON CONVOLUTION
        G_up = convolve(cur_img,-Ku,mode="constant",cval=0)
        G_up[0,:] = G_up[-1,:] = G_up[:,0] = G_up[:,-1] = 0

        G_vp = convolve(cur_img,-Kv,mode="constant",cval=0)
        G_vp[0,:] = G_vp[-1,:] = G_vp[:,0] = G_vp[:,-1] = 0

        G_rp = (2*u_p - N_up + 1)*G_up + (2*v_p - N_vp + 1)*G_vp
        G_tp = cur_img - prev_img



        ## SOLVE LEAST SQUARES PROBLEM
        X = np.array([
            [self.f*np.sum(G_vp*G_vp), -self.f*np.sum(G_up*G_vp), -w/2*np.sum(G_rp*G_vp)],
            [self.f*np.sum(G_vp*G_up), -self.f*np.sum(G_up*G_up), -w/2*np.sum(G_rp*G_up)],
            [self.f*np.sum(G_vp*G_rp), -self.f*np.sum(G_up*G_rp), -w/2*np.sum(G_rp*G_rp)]
        ])

        y = np.array([
            [np.sum(G_tp*G_vp)],
            [np.sum(G_tp*G_up)],
            [np.sum(G_tp*G_rp)]
        ])*(8*w/delta_t)

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y

        return b.flatten()

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

        N_vp = cur_img.shape[0]
        N_up = cur_img.shape[1]
        w = self.IW/N_up

        ## SEPERATED SOBEL KERNAL (U--DIRECTION)
        Ku_1 = np.array([[-1,0,1]]).reshape(3,1)
        Ku_2 = np.array([[ 1,2,1]]).reshape(1,3)


        ## SEPERATED SOBEL KERNAL (V--DIRECTION)
        Kv_1 = np.array([[ 1,2,1]]).reshape(3,1)
        Kv_2 = np.array([[-1,0,1]]).reshape(1,3)


        ## PRE-ALLOCATE INTENSITY GRADIENTS
        G_up = np.zeros((N_vp,N_up))
        G_vp = np.zeros((N_vp,N_up))
        G_rp = np.zeros((N_vp,N_up))
        G_tp = np.zeros((N_vp,N_up))


        ## CALCULATE IMAGE GRADIENTS
        for v_p in range(1, N_vp-1): 
            for u_p in range(1, N_up-1):
                G_up[v_p,u_p] = (Ku_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Ku_1)))).item()
                G_vp[v_p,u_p] = (Kv_2.dot((cur_img[v_p-1:v_p+2,u_p-1:u_p+2].dot(Kv_1)))).item()
                G_rp[v_p,u_p] = (2*u_p - N_up + 1)*G_up[v_p,u_p] + (2*v_p - N_vp + 1)*G_vp[v_p,u_p]
                G_tp[v_p,u_p] = cur_img[v_p,u_p] - prev_img[v_p,u_p]


        ## SOLVE LEAST SQUARES PROBLEM
        X = np.array([
            [self.f*np.sum(G_vp*G_vp), -self.f*np.sum(G_up*G_vp), -w/2*np.sum(G_rp*G_vp)],
            [self.f*np.sum(G_vp*G_up), -self.f*np.sum(G_up*G_up), -w/2*np.sum(G_rp*G_up)],
            [self.f*np.sum(G_vp*G_rp), -self.f*np.sum(G_up*G_rp), -w/2*np.sum(G_rp*G_rp)]
        ])

        y = np.array([
            [np.sum(G_tp*G_vp)],
            [np.sum(G_tp*G_up)],
            [np.sum(G_tp*G_rp)]
        ])*(8*w/delta_t)

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y

        return b.flatten()
   
    def Image_Subsample(self,image,subsample_level=0):

        convolve_array = np.array([
            [0.25,0.25],
            [0.25,0.25]
        ])
        image_downsampled = image

        for _ in range(0,subsample_level):

            image_downsampled = convolve(image_downsampled,convolve_array)[:image.shape[0]:2,:image.shape[1]:2]

        return image_downsampled

    def OpticalFlow_Writer(self,OF_Calc_Func,SubSample_Level=0):
        """
        Calculates optical flow values for all images in log file and appends them to log file
        """        

        Theta_x_est_arr = [0.0]
        Theta_y_est_arr = [0.0]
        Tau_est_arr = [0.0]
        
        ## SUB-SAMPLE IMAGES
        if SubSample_Level > 0:
            self.N_up = self.N_up//(2**SubSample_Level)
            self.N_vp = self.N_vp//(2**SubSample_Level)
            
            Image_list = []
            for ii,image in enumerate(self.Image_array):
                image = self.Image_Subsample(image,subsample_level=SubSample_Level)
                self.Data_df.iloc[ii,-1] = np.array2string(image,separator = ' ').replace('\n','').replace('[','').replace(']','') # Convert array to into string
                Image_list.append(image)
            self.Image_array = np.asarray(Image_list)
            
        with trange(1,self.n_imgs) as n:
            for ii in n:

                ## COLLECT CURRENT AND PREVIOUS IMAGE
                prev_img = self.Image_array[ii-1]
                cur_img = self.Image_array[ii]
                
                ## CALCULATE TIME BETWEEN IMAGES
                t_prev = self.grabState('t',ii-1)
                t_cur = self.grabState('t',ii)
                t_delta = t_cur - t_prev

                ## CALCULATE OPTICAL FLOW VALUES
                Theta_x_est,Theta_y_est,Theta_z_est = OF_Calc_Func(cur_img,prev_img,t_delta)

                Theta_x_est = np.clip(Theta_x_est,-20,20).round(2)
                Theta_y_est = np.clip(Theta_y_est,-20,20).round(2)
                Tau_est = np.clip(1/Theta_z_est,0,10).round(2)

                ## RECORD ESTIMATED VISUAL CUE VALUES
                Theta_x_est_arr.append(Theta_x_est)
                Theta_y_est_arr.append(Theta_y_est)
                Tau_est_arr.append(Tau_est)


        ## APPEND COMPILED DATA TO DATAFRAME
        self.df = self.Data_df.iloc[:,:-1]
        self.df['Tau_est'] = Tau_est_arr
        self.df['Theta_x_est'] = Theta_x_est_arr
        self.df['Theta_y_est'] = Theta_y_est_arr
        self.df['Camera_Data'] = self.Data_df.iloc[:,-1]

        if SubSample_Level == 0:
            self.df.to_csv(self.FilePath,index=False)
        else:
            self.FileName = self.FileName + f"--SSL_{SubSample_Level:0d}"
            self.FileDir = os.path.join(self.LogDir,self.FileName)   
            self.FilePath = os.path.join(self.FileDir,self.FileName + ".csv")    
            self.ConfigPath = os.path.join(self.FileDir,"Config.yaml")

            if not os.path.exists(self.FileDir):
                os.mkdir(self.FileDir)

            ## WRITE CSV AND CONFIG TO NEW DIRECTORY
            self.df.to_csv(self.FilePath,index=False)
            self.Write_Config_File()

        self.LoadData()
        self.Load_Config_File()



        

if __name__ == '__main__':

    Parser = DataParser(FileName="D_1.0--V_perp_0.5--V_||_1.0--L_0.125") 
    # Parser.OpticalFlow_Writer(Parser.OF_Calc_PyOpt,SubSample_Level=0)
    Parser.DataOverview(frame_idx=150,save_fig=True)
    # Parser.DataOverview_MP4(n=10,frame_limit=None)
    # Parser.OpticalFlow_MP4(n=10) 

    # Parser.Generate_Pattern(L=0.125,Surf_width=8,Surf_Height=16,save_img=True)
   