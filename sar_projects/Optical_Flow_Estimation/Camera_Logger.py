#!/usr/bin/env python3
import rospy
import numpy as np
import yaml
import csv
import sys
import os
import message_filters

from sensor_msgs.msg import Image,CameraInfo
from sar_msgs.msg import SAR_StateData,SAR_FlipData,SAR_ImpactData,SAR_MiscData
from sar_msgs.srv import ModelMove
from rosgraph_msgs.msg import Clock

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_projects'))
# sys.path.insert(1,BASE_PATH)



class CameraLogger:

    def __init__(self,FolderName,FileDir,D_perp,V_perp,V_parallel,y_offset=0.0,t_delta=1.0):
        
        rospy.init_node('Camera_Logger')

        ## INIT LOGGING PARAMETERS
        self.FileDir = FileDir
        self.LogDir = f"{BASE_PATH}/sar_projects/Optical_Flow_Estimation/local_logs/{FolderName}"
        self.FileDir = os.path.join(self.LogDir,self.FileDir)   
        self.CSV_Path = os.path.join(self.FileDir,"Cam_Data.csv")    
        self.Config_Path = os.path.join(self.FileDir,"Config.yaml")

        self.Logging_Flag = False
        
        ## GENERATE LOG DIRECTORY
        if not os.path.exists(self.FileDir):
            os.makedirs(self.FileDir,exist_ok=True)

        self.Create_CSV(self.CSV_Path)

        ## INIT FLIGHT CONDITION VALUES
        self.D_perp_0 = D_perp
        self.V_perp_0 = V_perp
        self.V_parallel_0 = V_parallel
        self.y_offset_0 = y_offset
        self.t_delta = t_delta

        np.set_printoptions(threshold = sys.maxsize) # Print full string without truncation

        ## PRE-INIT TIME VALUES
        self.t = 0.0
        self.t_0 = np.nan
        self.Init_Values_Flag = False

    
        ## DATA SUBSCRIBERS
        Image_Sub = message_filters.Subscriber("/SAR_Internal/camera/image_raw",Image)
        Info_Sub = message_filters.Subscriber("/SAR_Internal/camera/camera_info",CameraInfo)
        State_Sub = message_filters.Subscriber("/SAR_DC/StateData",SAR_StateData)
        self.ModelMove_Service = rospy.ServiceProxy('/ModelMovement',ModelMove)

        ## VERIFY ROS TOPICS
        print("Waiting for messages...")
        rospy.wait_for_message("/SAR_Internal/camera/image_raw",Image)
        rospy.wait_for_message("/SAR_Internal/camera/camera_info",CameraInfo)
        rospy.wait_for_message("/SAR_DC/StateData",SAR_StateData)
        print("Messages received")


        ## CREATE TIME SYNCHRONIZER FOR ROS TOPICS
        Approx_Time_Sync = message_filters.ApproximateTimeSynchronizer([Image_Sub,Info_Sub,State_Sub],slop=0.005,queue_size=500)
        Approx_Time_Sync.registerCallback(self.Data_Callback)

        while self.Init_Values_Flag == False:
            print("Waiting for callback...")

        self.save_Config_File()
        self.Model_Move_Command()
        self.Begin_Logging()

    def Data_Callback(self,Img_msg,Cam_Info_msg,StateData_msg):
        
        ## IMAGE DATA
        self.Camera_raw = np.frombuffer(Img_msg.data,np.uint8)      # 1D array to package into CSV   

        ## CAMERA_DATA
        self.N_up = Cam_Info_msg.width 
        self.N_vp = Cam_Info_msg.height

        ## STATE DATA
        self.t = np.round(StateData_msg.header.stamp.to_sec(),2)          # Sim time [s]

        self.pos = np.round([StateData_msg.Pose.position.x,
                             StateData_msg.Pose.position.y,
                             StateData_msg.Pose.position.z],3)

        self.vel = np.round([StateData_msg.Twist.linear.x,
                             StateData_msg.Twist.linear.y,
                             StateData_msg.Twist.linear.z],3)

        self.eul = np.round([StateData_msg.Eul.x,
                             StateData_msg.Eul.y,
                             StateData_msg.Eul.z],3)

        self.omega = np.round([StateData_msg.Twist.angular.x,
                               StateData_msg.Twist.angular.y,
                               StateData_msg.Twist.angular.z],3)

        self.quat = np.round([StateData_msg.Pose.orientation.x,
                              StateData_msg.Pose.orientation.y,
                              StateData_msg.Pose.orientation.z,
                              StateData_msg.Pose.orientation.w],3)

        ## VISUAL STATES
        self.Tau = np.round(StateData_msg.Tau,3)
        self.Theta_x = np.round(StateData_msg.Theta_x,3)
        self.Theta_y = np.round(StateData_msg.Theta_y,3)
        self.D_perp = np.round(StateData_msg.D_perp,3) 


        self.Init_Values_Flag = True


    ## LOG IF WITHIN RANGE OF LANDING SURFACE
        if self.Logging_Flag == True:

            if (0.1 < (self.t-self.t_0) <= self.t_delta+0.1 and self.D_perp >= 0.05):

                self.Append_CSV()
                print(f"Recording... Time: {self.t-self.t_0:.2f}")
            
            elif (self.t-self.t_0) >= self.t_delta + 0.1 or self.D_perp < 0.05:
                print(f"Finished Recording... Current Time: {self.t-self.t_0:.2f}")
                sys.exit()
               


    def save_Config_File(self):

        data = dict(
            IMAGE_SETTINGS = dict(
                N_up = self.N_up,
                N_vp = self.N_vp,
                f = 0.66e-3,
                FOV = 82.22,
                IW = 1.152e-3
                ),

            FLIGHT_CONDITIONS = dict(
                D_perp = self.D_perp_0,
                V_perp = self.V_perp_0,
                V_parallel = self.V_parallel_0,
                y_offset = self.y_offset_0,
            )
        )

        with open(self.Config_Path, 'w') as outfile:
            yaml.dump(data,outfile,default_flow_style=False,sort_keys=False)

    def Model_Move_Command(self,):

        Cam_X_Offset = 0.027    # [m]
        Cam_Z_Offset = 0.008    # [m]
        Plane_pos = 2.0         # [m]
    
        ## DATA SUBSCRIBERS
        rospy.Subscriber("/SAR_Internal/camera/image_raw",Image,self.CameraCallback,queue_size=500)
        rospy.Subscriber("/SAR_DC/StateData",SAR_StateData,self.SAR_StateDataCallback,queue_size=1)

        rospy.wait_for_service('/ModelMovement',timeout=1)
        service = rospy.ServiceProxy('/ModelMovement', ModelMove)
        service(Move_srv)

    def Begin_Logging(self):
        self.t_0 = self.t
        self.Logging_Flag = True

    def Create_CSV(self,FilePath):
        """Create CSV file that log data will be written to

        Args:
            FilePath (str): Filepath of CSV
        """        

        with open(FilePath,mode = 'w') as logfile:
            writer = csv.writer(logfile, delimiter = ',',quotechar = '"',quoting = csv.QUOTE_MINIMAL)
            writer.writerow([
                't',
                'x','y','z',
                'vx','vy','vz',
                'D_perp','Tau','Theta_x','Theta_y',
                'wx','wy','wz',
                'qx','qy','qz','qw',
                'Camera_Data'
                ])

            print(f'\nLogging file {self.FileDir} was successfully created\n')

    def Append_CSV(self):
        """Appends current state data and camera data to CSV file

        Args:
            FilePath (str): Filepath of CSV
        """        
        
        ## RENAME VARIABLES WITH CLEAR NAMES
        x,y,z = self.pos
        vx,vy,vz = self.vel

        qx,qy,qz,qw = self.quat
        wx,wy,wz = self.omega

        D_perp = self.D_perp
        Tau = self.Tau
        Theta_x = self.Theta_x
        Theta_y = self.Theta_y

        
        ## CLEAN CAMERA STRING
        Camera_data = np.array2string(self.Camera_raw,separator = ' ').replace('\n','').replace('[','').replace(']','') # Convert array to into string
    
        ## LOG DATA
        with open(self.CSV_Path,mode = 'a') as logfile:
            writer = csv.writer(logfile, delimiter = ',', quotechar = '"', quoting = csv.QUOTE_MINIMAL)
            writer.writerow([
            self.t,
            x,y,z,
            vx,vy,vz,
            D_perp,Tau,Theta_x,Theta_y,
            wx,wy,wz,
            qx,qy,qz,qw,
            Camera_data,
            ])

        
        self.t = np.round(Cam_msg.header.stamp.to_sec(),4)      # Sim time
        self.Camera_raw = np.frombuffer(Cam_msg.data,np.uint8)  # 1D array to package into CSV         
        self.Append_CSV()



    def SAR_StateDataCallback(self,StateData_msg):
        """Callback which receives current state data over ROS topic.

        Args:
            StateData_msg (_type_): Ros msg of camera data
        """        

        self.pos = np.round([StateData_msg.Pose.position.x,
                             StateData_msg.Pose.position.y,
                             StateData_msg.Pose.position.z],3)



if __name__ == '__main__':

    ## TRANSLATION FLOW
    # D_perp,V_parallel,V_perp = (0.5,2.0,0.0)
    # D_perp,V_parallel,V_perp = (0.8,1.6,0.0)

    ## DIVERGENT FLOW
    D_perp,V_parallel,V_perp = (2.0,0.0,1.0)
    # D_perp,V_parallel,V_perp = (1.0,0.0,1.0)
    # D_perp,V_parallel,V_perp = (0.5,0.0,0.5)

    L = 0.25

    FolderName = "Check_Pattern_Divergent_Flow"
    FileName = f"D_{D_perp:.1f}--V_perp_{V_perp:.1f}--V_para_{V_parallel:.1f}--L_{L:.2f}"
    CameraLogger(FolderName,FileName,D_perp,V_perp,V_parallel,t_delta=1.5)  # Initialize class
    rospy.spin()            # Run Program until canceled