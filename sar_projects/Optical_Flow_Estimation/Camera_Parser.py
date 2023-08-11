import numpy as np
import pandas as pd
import os
import yaml
from tqdm import trange
from scipy.ndimage import convolve
import re


import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle

## ADD SAR_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_projects'))
sys.path.insert(1,BASE_PATH)        
np.set_printoptions(threshold = sys.maxsize) # Print full string without truncation


class DataParser:

    def __init__(self,FolderName,FileDir,SubSample_Level=0):
        """Main class that handles data analysis and image compilation.

        Args:
            FileName (str): Filename of logfile to analyze
        """        

        ## INIT LOGGING PARAMETERS
        self.LogDir = f"{BASE_PATH}/sar_projects/Optical_Flow_Estimation/local_logs/{FolderName}"
        self.FileDir = os.path.join(self.LogDir,FileDir)   

        self.CSV_Path = os.path.join(self.FileDir,f"Cam_Data--SSL_{SubSample_Level:0d}.csv")    
        self.Config_Path = os.path.join(self.FileDir,f"Cam_Config--SSL_{SubSample_Level:0d}.yaml")

        self.SubSample_Level = SubSample_Level
        self.L = float(re.findall("L_(\d+\.\d+)",FileDir)[0])

        if not os.path.exists(self.CSV_Path):

            Default_Config_Path = os.path.join(self.FileDir,f"Cam_Config.yaml")
            Default_CSV_Path = os.path.join(self.FileDir,f"Cam_Data.csv")

            self.Load_Config_File(Default_Config_Path)
            self.LoadData(Default_CSV_Path)
            
            self.Optical_Flow_Writer(self.OF_Calc_PyOpt,self.SubSample_Level)

        else:
            self.Load_Config_File(self.Config_Path)
            self.LoadData(self.CSV_Path)


            


# '/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/Optical_Flow_Estimation/local_logs/Check_Pattern_Divergent_Flow/D_2.0--V_perp_1.0--V_para_0.0--L_0.25/Cam_Config--SSL_0.yaml'
# '/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/Optical_Flow_Estimation/local_logs/Check_Pattern_Divergent_Flow/D_2.0--V_perp_1.0--V_para_0.0--L_0.25/Config--SSL_0.yaml'
        
    
    def LoadData(self,CSV_Path):
        """Loads data from the given log file
        """        

        self.Data_df = pd.read_csv(CSV_Path,quotechar='"',low_memory=False,comment="#")

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

    def Load_Config_File(self,ConfigPath):
        """Loads camera parameters from camera config file
        """        

        with open(ConfigPath, 'r') as file:
            self.YAML_data = yaml.safe_load(file)

        self.f = self.YAML_data['IMAGE_SETTINGS']['Focal_Length']          # Focal Length [m]
        self.IW = self.YAML_data['IMAGE_SETTINGS']['Image_Width']        # Image Sensor Width [m]
        self.IW = self.YAML_data['IMAGE_SETTINGS']['Image_Height']        # Image Sensor Width [m]

        self.N_up = self.YAML_data['IMAGE_SETTINGS']['N_up']    # IMAGE WIDTH IN PIXELS
        self.N_vp = self.YAML_data['IMAGE_SETTINGS']['N_vp']    # IMAGE HEIGHT IN PIXELS

        
    def Write_Config_File(self):
        """Updates pixel values in camera config file
        """        

        self.YAML_data['IMAGE_SETTINGS']['N_up'] = self.N_up
        self.YAML_data['IMAGE_SETTINGS']['N_vp'] = self.N_vp

        with open(self.Config_Path, 'w') as outfile:
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

    def DataOverview(self,n=10,frame_idx=1,save_fig=False,show_fig=True):
        """Generates a figure including image and optical cue plots.

        Args:
            n (int, optional): Number of optical flow vectors to show in image. Defaults to 10.
            frame_idx (int, optional): Frame to be shown. Defaults to 1.
            save_fig (bool, optional): Saves figure to image if true. Defaults to False.
        """        

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

        if show_fig == True:
            plt.show()



    def DataOverview_MP4(self,n=10,frame_limit=None):
        """Saves recorded images from camera into an MP4 file in the log directory.
        This includes video of camera recording as well as optical flow vectors calculated 
        via L-K method. As well as plots showing optical cue estimates and ground truth values.

        Args:
            n (int, optional): Number of optical flow vectors to be shown. Defaults to 10.
            frame_limit (int, optional): End frame to stop video on. Defaults to None.

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

        Text_ax = fig.add_subplot(gs[0,0])
        Text_ax.set_axis_off()
        textbox = Text_ax.text(0.5,0.5,"_",
            horizontalalignment='center',
            verticalalignment='center',
        )
        textbox.set_fontsize('medium')
        

        ## GENERATE IMAGE AXES
        Img_ax = fig.add_subplot(gs[1:,0])

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

            textbox.set_text(
                f"\n\n"
                f"Camera Parameters: \n"
                f"Frame: {i:03d} | FPS: {int(1/t_delta):03d} [Hz]\n\n"
                f"Surface Parameters: \n"
                f"L: {self.L:.2f} [m]\n\n"

                f"Movement Parameters: \n"
                fr"$V_\perp$: {self.vx[i]:.2f} [m/s] | $V_\parallel$: {-self.vz[i]:.2f} | $D_\perp$: {self.D_perp[i]:.2f} [m]"
                f"\n\n"

                f"Estimates: \n"
                f"Tau_est: {self.Tau_est[i]:.2f}    Tau: {self.Tau[i]:.2f}\n"
                f"Theta_x_est: {self.Theta_x_est[i]: .2f}    Theta_x: {self.Theta_x[i]:.2f}\n"
                f"Theta_y_est: {self.Theta_y_est[i]: .2f}    Theta_y: {self.Theta_y[i]:.2f}\n"
                )
                
            
            ## UPDATE IMAGE
            im.set_data(cur_img)
            
            ##  UPDATE MARKER LOCATIONS
            Tau_marker.set_data(self.t[i]-self.t.min(),self.Tau_est[i])
            Theta_x_marker.set_data(self.t[i]-self.t.min(),self.Theta_x_est[i])
            Theta_y_plot.set_data(self.t[i]-self.t.min(),self.Theta_y_est[i])

            ## PRINT PROGRESS
            print(f"Image: {i:03d}/{frame_limit:03d}")
            

            return im,Q,Tau_marker,Theta_x_marker,Theta_y_plot,textbox

        
        ani = animation.FuncAnimation(fig, update, interval=100, blit=False,frames=range(2,frame_limit))
        ani.save(f"{self.FileDir}/Data_Overview--SSL_{self.SubSample_Level:0d}.mp4",dpi=200)

    def OpticalFlow_MP4(self,n=10,frame_limit=None):
        """
            Saves recorded images from camera into an MP4 file in the log directory
        """        
        if frame_limit == None:
            frame_limit = self.n_imgs-1

        ## GENERATE FIGURE
        fig = plt.figure(figsize=(6,6))

        ## CALCULATE INITIAL OPTICAL FLOW VALUES
        t_cur = self.grabState(['t'],idx=1)
        t_prev = self.grabState(['t'],idx=0)
        t_delta = t_cur - t_prev

        prev_img = self.Image_array[0]
        cur_img = self.Image_array[1]
        # du_dt,dv_dt = self.Calc_OF_LK(cur_img,prev_img,t_delta)
        
        N_vp = cur_img.shape[0]
        N_up = cur_img.shape[1]
        U_p,V_p = np.meshgrid(np.arange(0,N_up,1),np.arange(0,N_vp,1))

        ## GENERATE IMAGE AXES
        Img_ax = fig.add_subplot(111)
        # Img_ax.set_title(fr"$V_\perp$: {self.vx[0]:.2f} [m/s] | $V_\parallel$: {self.vy[0]:.2f} | $D_\perp$: {self.D_perp[0]:.2f} [m]"
        #                      f"\n"
        #                      f"Frame: {0:03d} | FPS: {int(1/t_delta):03d} [Hz]")
        # IMAGE PLOT
        im = Img_ax.imshow(self.Image_array[0], 
                interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',)

        # OPTICAL FLOW QUIVER PLOT 
        # Q = Img_ax.quiver(
        #     U_p[1::n,1::n],V_p[1::n,1::n],
        #     -du_dt[1::n,1::n],-dv_dt[1::n,1::n], # Need negative sign for arrows to match correct direction
        #     color='lime')
        
        Img_ax.set_xticks([])
        Img_ax.set_yticks([])
        Img_ax.set_xticklabels([])
        Img_ax.set_yticklabels([])


        def update(i):
            
            ## UPDATE OPTICAL FLOW
            t_cur = self.grabState(['t'],idx=i)
            t_prev = self.grabState(['t'],idx=i-1)
            t_delta = t_cur - t_prev
            # du_dt,dv_dt = self.Calc_OF_LK(self.Image_array[i],self.Image_array[i-1],t_delta,n=10)
            # Q.set_UVC(-du_dt[1::n,1::n],-dv_dt[1::n,1::n])
            
            # ## UPDATE IMAGE
            # Img_ax.set_title(fr"$V_\perp$: {self.vx[i]:.2f} [m/s] | $V_\parallel$: {self.vy[i]:.2f} | $D_\perp$: {self.D_perp[i]:.2f} [m]"
            #                  f"\n"
            #                  f"Frame: {i:03d} | FPS: {int(1/t_delta):03d} [Hz]")
            im.set_data(self.Image_array[i])
           
            ## PRINT PROGRESS
            print(f"Image: {i:03d}/{frame_limit:03d}")

            return im,
        
        ani = animation.FuncAnimation(fig, update, interval=100, blit=False,frames=range(2,frame_limit))
        ani.save(f"{self.FileDir}/L-K_Test.mp4")
        
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

    def Generate_Surface_Pattern(self,Surf_width=2,Surf_Height=2,L_w=0.5,L_h=0.5,pixel_density=200,save_img=False,X_cam=0.0,D_cam=0.5):
        """Save show pattern and save image to file. As well, display camera boundaries for validation

        Args:
            Surf_width (int, optional): Width of pattern to preserve pixel density. Defaults to 2.
            Surf_Height (int, optional): Heigt of pattern to preserve pixel density. Defaults to 2.
            L_w (float, optional): Feature width of sin wave. Defaults to 0.5.
            L_h (float, optional): Feature height of sin wave. Defaults to 0.5.
            pixel_density (int, optional): Number of pixels per meter. Defaults to 100.
            save_img (bool, optional): Save image to file. Defaults to False.
            X_cam (float, optional): X location of camera. Defaults to 0.5.
            D_cam (float, optional): Distance of camera from pattern surface. Defaults to 0.5.
        """        

        ## GENERATE PATTERN BOUNDS
        x = np.linspace(-0.5*Surf_width, 0.5*Surf_width, pixel_density*Surf_width)
        y = np.linspace(-0.5*Surf_Height,0.5*Surf_Height,pixel_density*Surf_Height)
        X,Y = np.meshgrid(x,y)

        I_0 = 255   # Max image intensity
        T_w = L_w*2 # Convert feature size to sin period
        T_h = L_h*2 # Convert feature size to sin period

        ## GENERATE PATTERN
        I = I_0/2*np.cos(2*np.pi/T_w*X)*np.cos(2*np.pi/T_h*Y) + 255/2
        I = np.where(I < 128,0,255)

        ## GENERATE CAMERA BOUNDS
        Img_Width =  2*np.tan(self.FOV_rad/2)*D_cam
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
                f"{BASE_PATH}/crazyflie_projects/Optical_Flow_Estimation/Surface_Patterns/" + 
                f"Pattern-W_{Surf_width}-H_{Surf_Height}-Lw_{L_w:.02f}-Lh_{L_h:.02f}.png",
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

        """Calculate optical flow values with seperable convolution and integer optimizations for C-style.
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
    
    def OF_Calc_Exp(self,cur_img,prev_img,delta_t):

        """Calculate optical flow values with seperable convolution and integer optimizations for C-style.
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

        ## CALC DOT PRODUCTS
        G_vp_vp = np.sum(G_vp*G_vp)
        G_vp_up = np.sum(G_vp*G_up)
        G_vp_rp = np.sum(G_vp*G_rp)
        G_vp_tp = np.sum(G_vp*G_tp)

        G_up_up = np.sum(G_up*G_up) 
        G_up_rp = np.sum(G_up*G_rp)
        G_up_tp = np.sum(G_up*G_tp)

        G_rp_rp = np.sum(G_rp*G_rp)
        G_rp_tp = np.sum(G_rp*G_tp)


        ## SOLVE LEAST SQUARES PROBLEM
        X = np.array([
            [G_vp_vp, -G_vp_up, -self.IW/(2*N_up*self.f)*G_vp_rp],
            [G_vp_up, -G_up_up, -self.IW/(2*N_up*self.f)*G_up_rp],
            [G_vp_rp, -G_up_rp, -self.IW/(2*N_up*self.f)*G_rp_rp]
        ])

        y = np.array([
            [G_vp_tp],
            [G_up_tp],
            [G_rp_tp]
        ])*(8*self.IW/(self.f*N_up*delta_t))

        ## SOLVE b VIA PSEUDO-INVERSE
        b = np.linalg.pinv(X)@y

        return b.flatten()
   
    def Image_Subsample(self,image,subsample_level=0):
        """Downsamples the image by convolving with a (2x2) averaging kernel then dropping even rows. 
        This averages and reduces the resolution by half for each subsample level

        Args:
            image (np.array): Image to be downsampled
            subsample_level (int, optional): Number of subsampling passes. Defaults to 0.

        Returns:
            np.array: Returns downsampled image
        """        

        Avg_Kernel = np.array([
            [0.25,0.25],
            [0.25,0.25]
        ])
        image_downsampled = image

        for _ in range(0,subsample_level):

            image_downsampled = convolve(image_downsampled,Avg_Kernel) # Convolve image with averaging kernal
            image_downsampled = image_downsampled[:image.shape[0]:2,:image.shape[1]:2] # Drop even columns

        return image_downsampled

    def Optical_Flow_Writer(self,OF_Calc_Func,SubSample_Level):
        """Compiles optical cue values for all images in log file and appends them to log file.
        This function also takes in Subsampling level and will create a new log file.

        Args:
            OF_Calc_Func (func): Optical cue calculation function
            SubSample_Level (int, optional): Level of subsampling. Creates new log file and directory if not equal to zero. Defaults to 0.
        """        
            
        ## INITIAL OPTICAL CUE VALUES SINCE FIRST VALUES ARE UNDEFINED
        Theta_x_est_arr = [0.0]
        Theta_y_est_arr = [0.0]
        Tau_est_arr = [0.0]
        
        ## SUB-SAMPLE IMAGES
        if SubSample_Level > 0:
            self.N_up = self.N_up//(2**SubSample_Level)
            self.N_vp = self.N_vp//(2**SubSample_Level)
            
            Image_list = []
            print("Subsampling Images...")
            for ii,image in enumerate(self.Image_array):
                image = self.Image_Subsample(image,subsample_level=SubSample_Level)
                self.Data_df.iloc[ii,-1] = np.array2string(image,separator = ' ').replace('\n','').replace('[','').replace(']','') # Convert array to into string
                Image_list.append(image)
            self.Image_array = np.asarray(Image_list)
            print("Finished Subsampling Images.") 
            
        ## CALCULATE OPTICAL CUE ESTIMATES    
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



        self.CSV_Path = os.path.join(self.FileDir,f"Cam_Data--SSL_{SubSample_Level:0d}.csv")    
        self.Config_Path = os.path.join(self.FileDir,f"Cam_Config--SSL_{SubSample_Level:0d}.yaml")

        ## WRITE CSV AND CONFIG TO NEW DIRECTORY
        self.df.to_csv(self.CSV_Path,index=False)
        self.Write_Config_File()

        ## RELOAD COMPILED DATA AND LOG FILE
        self.LoadData(self.CSV_Path)
        self.Load_Config_File(self.Config_Path)



        

if __name__ == '__main__':

    FolderName = "Check_Pattern_Divergent_Flow"
    FileDir=f"D_2.0--V_perp_1.0--V_para_0.0--L_0.25"
    Parser = DataParser(FolderName,FileDir,SubSample_Level=0) 




    
    # Parser.DataOverview_MP4(n=8)
    Parser.OpticalFlow_MP4(n=8) 


    # L_list = [0.02,0.05,0.12,0.25,0.5,0.75,1.00,2.00]

    # for L in L_list:

    #     FileDir=f"D_2.0--V_perp_2.0--V_para_0.0--L_{L:.2f}"

    #     for SSL in  range(4):
    #         Parser = DataParser(FolderName,FileDir,SubSample_Level=SSL) 
    #         Parser.DataOverview_MP4(n=10,frame_limit=None)



    
    # # Parser.DataOverview(frame_idx=150,save_fig=True,show_fig=False)

    # L = 0.05
    # Parser.Generate_Surface_Pattern(L_w=L,L_h=L,Surf_width=8,Surf_Height=16,save_img=True)



    # img_cur = [
    #     5,8,4,6,1,8,7,0,7,0,
    #     0,0,0,5,0,0,4,2,4,2,
    #     2,4,8,8,3,1,0,1,0,1,
    #     0,6,0,8,2,9,8,5,8,5,
    #     7,1,9,6,1,5,5,3,5,3,
    #     8,2,0,3,1,3,8,1,8,1,
    #     3,0,8,8,0,7,6,1,6,1,
    #     8,0,8,1,9,9,3,5,3,5,
    #     0,6,0,8,2,9,8,5,8,5,
    #     7,1,9,6,1,5,5,3,5,3,
    #     ]
    # img_cur = np.array(img_cur).reshape(10,10)
    
    # img_prev = [
    #     9,2,2,2,9,7,6,8,6,8,
    #     8,1,3,8,2,2,2,9,2,9,
    #     2,6,4,4,1,5,8,9,8,9,
    #     2,6,1,0,5,3,3,4,3,4,
    #     8,5,4,2,9,3,9,8,9,8,
    #     8,2,9,3,0,7,3,2,3,2,
    #     0,4,3,3,8,0,4,6,4,6,
    #     1,0,8,7,6,8,5,7,5,7,
    #     2,6,1,0,5,3,3,4,3,4,
    #     8,5,4,2,9,3,9,8,9,8,
    #     ]
    # img_prev = np.array(img_prev).reshape(10,10)

    # Parser = DataParser(FolderName,FileDir,SubSample_Level=0) 
    # OF_vec = Parser.OF_Calc_Opt_Sep(img_cur,img_prev,0.01)
    # print(OF_vec)

    # OF_vec = Parser.OF_Calc_Exp(img_cur,img_prev,0.01)
    # print(OF_vec)

   