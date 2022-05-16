import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from sensor_msgs.msg import Image #import camera data
from crazyflie_msgs.msg import CF_StateData #import 'true' values

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
        self.d_ceil = 0
        self.prev_time = 0

        self.cam_sub = rospy.Subscriber("/CF_Internal/camera/image_raw",Image,self.Camera_cb,queue_size = 1)
        self.state_sub = rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size = 1)
            

    def Camera_cb(self,Cam_msg):

        self.time = np.round(Cam_msg.header.stamp.to_sec(),6) #grabbing timestamp
        self.Cur_img = np.frombuffer(Cam_msg.data,np.uint8).reshape(WIDTH_PIXELS,HEIGHT_PIXELS) #pulling in image to process
        self.logs()
       

    def CF_StateDataCallback(self,StateData_msg):
        
        self.Tau = np.round(StateData_msg.Tau,3)
        self.OFx = np.round(StateData_msg.OFx,3)
        self.OFy = np.round(StateData_msg.OFy,3)
        self.Tau_est_cpp = np.round(StateData_msg.Tau_est,3)
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
        
        fig = plt.figure(1)
        
        ## PLOT TAU VALUES
        ax1 = fig.add_subplot(211)
        ax1.plot(self.t_List,self.Tau_act_List,color = 'tab:blue',label = 'Tau')
        ax1.plot(self.t_List,self.Tau_cpp_est_List,color = 'r',linestyle = '--',label = 'Tau_est')
        ax1.grid()
        ax1.legend(loc='upper right')
        ax1.set_ylabel("Tau [s]")
        ax1.set_xlabel("Time [s]")

        ## PLOT ERROR
        Tau_act = np.array(self.Tau_act_List) #make lists into arrays to do math on them
        Tau_est = np.array(self.Tau_cpp_est_List)
        ax2 = fig.add_subplot(212,sharex = ax1)
        ax2.plot(self.t_List,(Tau_est - Tau_act),color = 'r',label = "Error in Tau")
        ax2.hlines(y =  0.05, xmin = 0, xmax = self.t_List[-1],linestyle = "--", linewidth = 2, color = 'k') #plot desired error bounds
        ax2.hlines(y = -0.05, xmin = 0, xmax = self.t_List[-1],linestyle = "--", linewidth = 2, color = 'k')
        ax2.vlines(x = (self.t_List[-1] - 1), ymin = -0.05, ymax = 0.05, linestyle = "--", linewidth = 2, color = "k")
        ax2.vlines(x = (self.t_List[-1]), ymin = -0.05, ymax = 0.05, linestyle = "--", linewidth = 2, color = "k")
        ax2.grid()
        ax2.legend(loc='upper right')
        ax2.set_ylabel("Error")
        ax2.set_xlabel("Time [s]")

        plt.show()


        ## OFX_Y PLOTS
        fig2 = plt.figure(2)
        #OFy Plots
        ax1 = fig2.add_subplot(221)
        ax1.plot(self.t_List,self.OFy_act_List, color = "tab:blue", label = "OFy")
        ax1.plot(self.t_List,self.OFy_est_List, color = "r", linestyle = "--", label = "OFy_est")
        ax1.grid()
        ax1.legend(loc='upper right')
        ax1.set_ylabel("OFy [rad/s]")
        ax1.set_xlabel("Time [s]")
        #OFy error plots
        ax2 = fig2.add_subplot(222,sharex = ax1)
        OFy_act = np.array(self.OFy_act_List) #make lists into arrays to do math on them
        OFy_est = np.array(self.OFy_est_List)
        ax2.plot(self.t_List,(OFy_est - OFy_act),color = "r",label = "Error in OFy")
        ax2.grid()
        ax2.legend(loc='upper right')
        ax2.set_ylabel("Error")
        ax2.set_xlabel("Time [s]")

        #OFx Plots
        ax3 = fig2.add_subplot(223)
        ax3.plot(self.t_List,self.OFx_act_List, color = "tab:blue", label = "OFx")
        ax3.plot(self.t_List,self.OFx_est_List, color = "r", linestyle = "--", label = "OFx_est")
        ax3.grid()
        ax3.legend(loc='upper right')
        ax3.set_ylabel("OFx [rad/s]")
        ax3.set_xlabel("Time [s]")
        #OFx error plots
        ax4 = fig2.add_subplot(224,sharex = ax3)
        OFy_act = np.array(self.OFy_act_List) #make lists into arrays to do math on them
        OFy_est = np.array(self.OFy_est_List)
        ax4.plot(self.t_List,(OFy_est - OFy_act),color = "r",label = "Error in OFy")
        ax4.grid()
        ax4.legend(loc='upper right')
        ax4.set_ylabel("Error")
        ax4.set_xlabel("Time [s]")


if __name__ == '__main__':

    CameraClass() #initialize the class when run
    rospy.spin() #run until this program is shutdown