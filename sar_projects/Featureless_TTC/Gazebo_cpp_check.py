import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from sensor_msgs.msg import Image #import camera data
from crazyflie_msgs.msg import CF_StateData #import 'true' values
from scipy import signal

## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160


class CameraClass:

    def __init__(self):

        rospy.init_node('Camera_alg', anonymous = True)#start nodes

        # Pre_init vars
        self.Tau_act_List = [0]
        self.Tau_cpp_est_List = [0]
        self.OFy_act_List = [0]
        self.OFy_est_List = [0]
        self.OFx_act_List = [0]
        self.OFx_est_List = [0]
        self.t_List = [0]

        self.Cur_img = np.array([])
        self.Prev_img = np.zeros([WIDTH_PIXELS,HEIGHT_PIXELS])
        self.Tau = 0
        self.OFx = 0
        self.OFy = 0
        self.Tau_est_cpp = 0
        self.OFy_est = 0
        self.OFx_est = 0
        self.d_ceil = rospy.get_param("/ENV_SETTINGS/Ceiling_Height") #init above 0,1 to prevent plot from popping up on startup
        self.prev_time = 0

        self.cam_sub = rospy.Subscriber("/CF_Internal/camera/image_raw",Image,self.Camera_cb,queue_size = 1)
        self.state_sub = rospy.Subscriber("/SAR_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size = 1)
            

    def Camera_cb(self,Cam_msg):

        self.time = np.round(Cam_msg.header.stamp.to_sec(),6) #grabbing timestamp
        self.Cur_img = np.frombuffer(Cam_msg.data,np.uint8).reshape(WIDTH_PIXELS,HEIGHT_PIXELS) #pulling in image to process
        self.logs()
       

    def CF_StateDataCallback(self,StateData_msg):
        
        self.Tau = np.round(StateData_msg.Tau,4)
        self.OFx = np.round(StateData_msg.OFx,3)
        self.OFy = np.round(StateData_msg.OFy,3)
        self.Tau_est_cpp = np.round(StateData_msg.Tau_est,4)
        self.OFy_est = np.round(StateData_msg.OFy_est,3)
        self.OFx_est = np.round(StateData_msg.OFx_est,3)
        self.d_ceil = np.round(StateData_msg.D_ceil,3)

    
    def logs(self):


        Tau_act = self.Tau
        OFy_act = self.OFy
        OFx_act = self.OFx
        
        print(f"Tau_act: {Tau_act:.3f} | Tau_est_cpp: {self.Tau_est_cpp}")
        print(f"OFy_act: {OFy_act:.3f} | OFy_est: {self.OFy_est:.3f}")
        print(f"OFx_act: {OFx_act:.3f} | OFx_est: {self.OFx_est:.3f}")
        print(f"Distance to Ceiling {self.d_ceil:.3f}\n")


        ## APPEN OPTICAL FLOW ESTIMATES TO LIST FOR PLOTTING
        self.Tau_act_List.append(Tau_act)
        self.Tau_cpp_est_List.append(self.Tau_est_cpp)
        self.OFy_act_List.append(OFy_act)
        self.OFy_est_List.append(self.OFy_est)
        self.OFx_act_List.append(OFx_act)
        self.OFx_est_List.append(self.OFx_est)
        self.t_List.append(self.time)

        if (self.d_ceil < 0.1):
                self.LoggingFlag = False
                self.plotter()

    def plotter(self):
        
        #Generate arrays to format plots better
        Time_arr = np.array(self.t_List)
        Tau_act = np.array(self.Tau_act_List)
        Tau_est = np.array(self.Tau_cpp_est_List)
        OFy_act = np.array(self.OFy_act_List)
        OFx_act = np.array(self.OFx_act_List)
        OFy_est = np.array(self.OFy_est_List)
        OFx_est = np.array(self.OFx_est_List)

        ## NORMALIZE X AXIS
        Time = np.zeros_like(Time_arr)
        # self.Time[0] = 0
        for n in range(2,Time_arr.size):
            dt = Time_arr[n] - Time_arr[n-1]
            Time[n] = Time[n-1] + dt


        ## Y AXIS SCALING SET
        

        fig = plt.figure(1)
        
        ## PLOT TAU VALUES
        fig.suptitle("Tau Estimates (Percent of Data Used = 0.1%)", fontsize=26)
        ax1 = fig.add_subplot(211)
        ax1.plot(Time[3:],Tau_act[3:],color = 'tab:blue',label = 'Tau')
        ax1.plot(Time[3:],Tau_est[3:],color = 'r',linestyle = '--',label = 'Tau Estimate')
        ax1.hlines(y = 0, xmin = Time[-1] - 5, xmax = Time[-1] + 5, linestyle = "-", linewidth = 1, color = 'k')
        ax1.grid()
        ax1.legend(loc='best',fontsize=16)
        ax1.set_ylabel("Tau [s]",fontsize=16)
        ax1.set_xlabel("Time [s]",fontsize=16)
        ax1.tick_params(axis='both', which='major', labelsize=14)
        ax1.tick_params(axis='both', which='minor', labelsize=14)
        ax1.set_xlim([0,Time[-1] + 0.1])
        

        ## PLOT ERROR
        ax2 = fig.add_subplot(212,sharex = ax1)
        ax2.plot(Time[3:],(Tau_est[3:] - Tau_act[3:]),color = 'r',label = "Error in Tau")

        #plot desired error bounds
        ax2.hlines(y = 0, xmin = Time[1:] - 5, xmax = Time[-1] + 5, linestyle = "-", linewidth = 1, color = 'k')
        ax2.hlines(y =  0.05, xmin = Time[-1] - 1, xmax = Time[-1],linestyle = "--", linewidth = 2, color = 'k') 
        ax2.hlines(y = -0.05, xmin = Time[-1] - 1, xmax = Time[-1],linestyle = "--", linewidth = 2, color = 'k')
        ax2.vlines(x = (Time[-1] - 1), ymin = -0.05, ymax = 0.05, linestyle = "--", linewidth = 2, color = "k")
        ax2.vlines(x = (Time[-1]), ymin = -0.05, ymax = 0.05, linestyle = "--", linewidth = 2, color = "k")

        ax2.hlines(y = 0, xmin = Time[-1] - 5, xmax = Time[-1] + 5, linestyle = "-", linewidth = 1, color = 'k')
        ax2.grid()
        ax2.legend(loc='best',fontsize=16)
        ax2.set_ylabel("Error",fontsize=16)
        ax2.set_xlabel("Time [s]",fontsize=16)
        ax2.tick_params(axis='both', which='major', labelsize=14)
        ax2.tick_params(axis='both', which='minor', labelsize=14)
        ax2.set_xlim([0,Time[-1] + 0.1])
        ax2.set_ylim([-0.45,0.45])


        ## OFX_Y PLOTS
        fig2 = plt.figure(2)
        #OFy Plots
        ax1 = fig2.add_subplot(221)
        ax1.plot(Time[1:],OFy_act[1:], color = "tab:blue", label = "OFy")
        ax1.plot(Time[1:],OFy_est[1:], color = "r", linestyle = "--", label = "OFy Estimate")
        ax1.grid()
        ax1.legend(loc='best',fontsize=16)
        ax1.set_ylabel("OFy [rad/s]",fontsize=16)
        ax1.set_xlabel("Time [s]",fontsize=16)
        #OFy error plots
        ax2 = fig2.add_subplot(222,sharex = ax1)
        ax2.plot(Time[1:],(OFy_est[1:] - OFy_act[1:]),color = "r",label = "Error in OFy")
        ax2.grid()
        ax2.legend(loc='best',fontsize=16)
        ax2.set_ylabel("Error",fontsize=16)
        ax2.set_xlabel("Time [s]",fontsize=16)

        #OFx Plots
        ax3 = fig2.add_subplot(223)
        ax3.plot(Time[1:],OFx_act[1:], color = "tab:blue", label = "OFx")
        ax3.plot(Time[1:],OFx_est[1:], color = "r", linestyle = "--", label = "OFx Estimate")
        ax3.grid()
        ax3.legend(loc='best',fontsize=16)
        ax3.set_ylabel("OFx [rad/s]",fontsize=16)
        ax3.set_xlabel("Time [s]",fontsize=16)
        #OFx error plots
        ax4 = fig2.add_subplot(224,sharex = ax3)
        ax4.plot(Time[1:],(OFx_est[1:] - OFx_act[1:]),color = "r",label = "Error in OFx")
        ax4.grid()
        ax4.legend(loc='best',fontsize=16)
        ax4.set_ylabel("Error",fontsize=16)
        ax4.set_xlabel("Time [s]",fontsize=16)

        plt.show()

        # self.filters() #call filter experiments

    def filters(self):

        Time_arr = np.array(self.t_List)
        Tau_act = np.array(self.Tau_act_List) #make lists into arrays to do math on them
        Tau_est = np.array(self.Tau_cpp_est_List)

        ## NORMALIZE X AXIS
        Time = np.zeros_like(Time_arr)
        # self.Time[0] = 0
        for n in range(2,Time_arr.size):
            dt = Time_arr[n] - Time_arr[n-1]
            Time[n] = Time[n-1] + dt

        ## EXPONENTIAL MOVING AVERAGE FILTER y[n] = a*x[n] + (1-a)*y[n-1]
        EMA = np.zeros_like(Tau_est)
        DEMA = np.zeros_like(Tau_est)
        a = 0.9 #
        EMA[0] = Tau_est[0]

        for n in range(1,EMA.shape[0]):
            EMA[n] = a*Tau_est[n-1] + (1-a)*EMA[n-1]
            DEMA[n] = 2*EMA[n-1] - (a*Tau_est[n-1] + (1-a)*EMA[n-1])

        ## BUTTERWORTH LOW PASS FILTER
        fs = 50 # FPS
        fc = 10 #cutoff frequency HZ
        w = fc/(fs/2)
        b,a = signal.butter(1,w,"low",analog = False)
        print(f"a coef: {a}\n") 
        print(f"b coef: {b}\n") 
        output = signal.filtfilt(b,a,Tau_est)
        fig = plt.figure(3)        

        ## TAU PLOT
        ax1 = fig.add_subplot(211)
        ax1.plot(Time[1:],output[1:], label = "filtered",color = "g",linestyle = "--")
        ax1.plot(Time[1:],Tau_est[1:], label = "Tau_est",color = "r",linestyle = "--")
        ax1.plot(Time[1:],EMA[1:], label = "EMA",color = "m",linestyle = "--")
        ax1.plot(Time[1:],Tau_act[1:],color = 'tab:blue',label = 'Tau')
        ax1.grid()
        ax1.legend()

        # ERROR PLOT
        ax2 = fig.add_subplot(212,sharex = ax1)
        ax2.plot(Time[1:], Tau_est[1:] - Tau_act[1:], color = "r",label = "Estimate Error")
        ax2.plot(Time[1:], output[1:] - Tau_act[1:], color = "g", linestyle = "--", label = "LPF Error")
        ax2.plot(Time[1:], EMA[1:] - Tau_act[1:], color = "b", linestyle = "--", label = "EMA Error")
        ax2.plot(Time[1:], DEMA[1:] - Tau_act[1:], color = "k", linestyle = "--", label = "DEMA Error")
        ax2.hlines(y =  0.05, xmin = Time[-1] - 1, xmax = Time[-1],linestyle = "--", linewidth = 2, color = 'k') #plot desired error bounds
        ax2.hlines(y = -0.05, xmin = Time[-1] - 1, xmax = Time[-1],linestyle = "--", linewidth = 2, color = 'k')
        ax2.vlines(x = (Time[-1] - 1), ymin = -0.05, ymax = 0.05, linestyle = "--", linewidth = 2, color = "k")
        ax2.vlines(x = (Time[-1]), ymin = -0.05, ymax = 0.05, linestyle = "--", linewidth = 2, color = "k")
        ax2.grid()
        ax2.legend()
        plt.show()



if __name__ == '__main__':

    CameraClass() #initialize the class when run
    rospy.spin() #run until this program is shutdown