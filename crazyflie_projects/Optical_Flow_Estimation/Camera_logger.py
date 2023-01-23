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


class CameraLogger:

    def __init__(self):
        
        rospy.init_node('Camera_Data', anonymous = True)

        ## INIT LOGGING PARAMETERS
        self.Username = getpass.getuser()
        self.Path = f'/home/{self.Username}/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Optical_Flow_Estimation/local_logs' 
        # self.FileName = input("Input the name of the log file:\n")
        self.FileName = "ExampleLog.csv"
        self.Path = os.path.join(self.Path,self.FileName)
        self.Create_csv()

        np.set_printoptions(threshold = sys.maxsize) # Print full string without truncation
        self.Camera_raw = np.array([])

        ## INITIALIZE STATE VALUES
        self.t = 0.0
        self.t_prev = 0.0

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
        rospy.Subscriber("/CF_Internal/camera/image_raw",Image,self.CameraCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size=1)

    def Create_csv(self):

        with open(self.Path,mode = 'w') as logfile:
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

    def Append_data(self):
        
        
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
        if(0.05 < D_perp < 2.0): 

            ## CLEAN CAMERA STRING
            Camera_data = np.array2string(self.Camera_raw,separator = ' ').replace('\n','').replace('[','').replace(']','') # Convert array to into string
        
            with open(self.Path,mode = 'a') as logfile:
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
        
        self.t = np.round(Cam_msg.header.stamp.to_sec(),4)      # Sim time
        self.Camera_raw = np.frombuffer(Cam_msg.data,np.uint8)  # 1D array to package into CSV 
        
        ## In order to filter need to reshape and filter then go back to 1D
        self.Append_data()

    def CF_StateDataCallback(self,StateData_msg):

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
       
        self.t_prev = self.t # Save t value for next callback iteration

        
    

if __name__ == '__main__':

    CameraLogger() #initialize the class when run
    rospy.spin() #run until this program is shutdown