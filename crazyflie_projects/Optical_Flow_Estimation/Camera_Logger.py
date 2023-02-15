#!/usr/bin/env python3
import rospy
import numpy as np
import yaml
import csv
import sys
import os
import message_filters


from sensor_msgs.msg import Image,CameraInfo
from crazyflie_msgs.msg import CF_StateData,CF_FlipData,CF_ImpactData,CF_MiscData
from crazyflie_msgs.srv import ModelMove,ModelMoveRequest


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(1,BASE_PATH)



class CameraLogger:

    def __init__(self,FileName,D_perp,V_perp,V_parallel,y_offset=-2):
        
        rospy.init_node('Camera_Logger')

        ## INIT LOGGING PARAMETERS
        self.FileName = FileName
        self.LogDir = f"{BASE_PATH}/crazyflie_projects/Optical_Flow_Estimation/local_logs/"
        self.FileDir = os.path.join(self.LogDir,self.FileName)   
        self.FilePath = os.path.join(self.FileDir,self.FileName + ".csv")    
        self.ConfigPath = os.path.join(self.FileDir,"Config.yaml")

        self.Logging_Flag = False
        
        ## GENERATE LOG DIRECTORY
        if not os.path.exists(self.FileDir):
            os.makedirs(self.FileDir,exist_ok=True)

        self.Create_CSV(self.FilePath)

        ## INIT FLIGHT CONDITION VALUES
        self.D_perp_0 = D_perp
        self.V_perp_0 = V_perp
        self.V_parallel_0 = V_parallel
        self.y_offset_0 = y_offset

        np.set_printoptions(threshold = sys.maxsize) # Print full string without truncation

        ## PRE-INIT TIME VALUES
        self.t = 0.0
        self.t_0 = np.nan
        self.Init_Values_Flag = False

    
        ## DATA SUBSCRIBERS
        Image_Sub = message_filters.Subscriber("/CF_Internal/camera/image_raw",Image)
        Info_Sub = message_filters.Subscriber("/CF_Internal/camera/camera_info",CameraInfo)
        State_Sub = message_filters.Subscriber("/CF_DC/StateData",CF_StateData)
        self.ModelMove_Service = rospy.ServiceProxy('/ModelMovement',ModelMove)

        ## VERIFY ROS TOPICS
        print("Waiting for messages...")
        rospy.wait_for_message("/CF_Internal/camera/image_raw",Image)
        rospy.wait_for_message("/CF_Internal/camera/camera_info",CameraInfo)
        rospy.wait_for_message("/CF_DC/StateData",CF_StateData)
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
        if 0.1 < (self.t-self.t_0) <= 1.1 and self.Logging_Flag == True:
            self.Append_CSV()
            print(f"Recording... Time: {self.t-self.t_0:.2f}")

        elif (self.t-self.t_0) >= 1.1:
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

        with open(self.ConfigPath, 'w') as outfile:
            yaml.dump(data,outfile,default_flow_style=False,sort_keys=False)

    def Model_Move_Command(self,):

        Cam_offset = 0.027  # [m]
        Plane_pos = 2.0     # [m]
    
        ## RESET POSITION AND VELOCITY
        Move_srv = ModelMoveRequest()
        
        Move_srv.Pos_0.x = Plane_pos - self.D_perp_0 - Cam_offset
        Move_srv.Pos_0.y = self.y_offset_0
        Move_srv.Pos_0.z = 0.0

        Move_srv.Vel_0.x = self.V_perp_0
        Move_srv.Vel_0.y = self.V_parallel_0
        Move_srv.Vel_0.z = 0.0

        Move_srv.Accel_0.x = 0.0
        Move_srv.Accel_0.y = 0.0
        Move_srv.Accel_0.z = 0.0

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

            print(f'\nLogging file {self.FileName} was successfully created\n')

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
        with open(self.FilePath,mode = 'a') as logfile:
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

        



if __name__ == '__main__':

    D_perp = 0.5
    V_perp = 0.0
    V_parallel = 4.0
    y_offset = -4.0

    FileName = f"Theta_y--Vy_{V_parallel:.1f}--D_{D_perp:.1f}--L_0.25"
    CameraLogger(FileName,D_perp=2.0,V_perp=0.0,V_parallel=4.0,y_offset=-4)  # Initialize class
    rospy.spin()            # Run Program until canceled