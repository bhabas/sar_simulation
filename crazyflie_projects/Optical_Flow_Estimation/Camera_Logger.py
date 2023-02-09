#!/usr/bin/env python3
import rospy
import numpy as np
import getpass
import csv
import sys
import os

from sensor_msgs.msg import Image
from crazyflie_msgs.msg import CF_StateData,CF_FlipData,CF_ImpactData,CF_MiscData
from rosgraph_msgs.msg import Clock

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(1,BASE_PATH)



class CameraLogger:

    def __init__(self):
        
        rospy.init_node('Camera_Data', anonymous = True)

        ## INIT LOGGING PARAMETERS
        # self.FileName = input("Input the name of the log file:\n")
        self.FileName = "Theta_y--Vy_1.0--D_0.5"
        self.LogDir = f"{BASE_PATH}/crazyflie_projects/Optical_Flow_Estimation/local_logs/{self.FileName}"
        
        ## GENERATE LOG DIRECTORY
        if not os.path.exists(self.LogDir):
            os.makedirs(self.LogDir,exist_ok=True)

        self.FilePath = os.path.join(self.LogDir,self.FileName) + ".csv"
        self.Create_csv(self.FilePath)





        ## INITIALIZE CAMERA VALUES
        self.Camera_raw = np.array([])
        np.set_printoptions(threshold = sys.maxsize) # Print full string without truncation

        ## INITIALIZE STATE VALUES
        self.t = 0.0

        self.pos = [0,0,0]
        self.vel = [0,0,0]

        self.quat = [0,0,0,1]
        self.eul = [0,0,0]
        self.omega = [0,0,0]

        ## INITIALIZE GROUND TRUTH VALUES
        self.Tau = 0.0
        self.Theta_x = 0.0
        self.Theta_y = 0.0
        self.D_perp = 0.0 
        
    
        ## DATA SUBSCRIBERS
        rospy.Subscriber("/CF_Internal/camera/image_raw",Image,self.CameraCallback,queue_size=500)
        rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size=1)

    def Create_csv(self,FilePath):
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

        ## LOG IF WITHIN RANGE OF LANDING SURFACE
        if(0.1 < y < 1.2): 

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




    def CameraCallback(self,Cam_msg):
        """Callback from receiving camera data over ROS topic. This function reads time from the msg
        and initiates logging to CSV for each message received.

        Args:
            Cam_msg (_type_): ROS msg of camera data
        """        
        
        self.t = np.round(Cam_msg.header.stamp.to_sec(),4)      # Sim time
        self.Camera_raw = np.frombuffer(Cam_msg.data,np.uint8)  # 1D array to package into CSV         
        self.Append_CSV()



    def CF_StateDataCallback(self,StateData_msg):
        """Callback which receives current state data over ROS topic.

        Args:
            StateData_msg (_type_): Ros msg of camera data
        """        

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
       

        
    

if __name__ == '__main__':

    CameraLogger()  # Initialize class
    rospy.spin()    # Run Program until canceled