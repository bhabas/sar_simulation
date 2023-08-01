#!/usr/bin/env python3
import rospy
import numpy as np
import getpass
import csv
import sys

from sensor_msgs.msg import Image
from sar_msgs.msg import SAR_StateData,SAR_TriggerData,SAR_ImpactData,SAR_MiscData
from rosgraph_msgs.msg import Clock


class CameraParser:


    def __init__(self):
        
        rospy.init_node('Camera_Data', anonymous = True)#start nodes

        ## INITIALIZE STATE VALUES
        self.posCF = [0,0,0]
        self.velCF = [0,0,0]

        self.quatCF = [0,0,0,1]
        self.eulCF = [0,0,0]
        self.omegaCF = [0,0,0]
        self.eulCF = [0,0,0]

        self.Tau = 0.0
        self.OFx = 0.0
        self.OFy = 0.0
        self.d_ceil = 0.0 
        
        self.t = 0.0
        self.t_prev = 0.0

        self.Camera_raw = np.array([])

        #INIT PARAMETERS
        # self.h_ceiling = rospy.get_param("/ENV_SETTINGS/Ceiling_Height")

        self.Username = getpass.getuser()
        self.Path = f'/home/{self.Username}/catkin_ws/src/sar_simulation/crazyflie_projects/Featureless_TTC/local_logs' #store the logs in a folder for organization
        self.Filename = input("\ninput the name of the log file:\n") #wait for user input then create CSV with the given Filename
        self.Path = self.Path + '/' + self.Filename + '.csv'
        self.Create_csv()
        np.set_printoptions(threshold = sys.maxsize) #allows it to print the full string without truncation

        #COLLECT MISC. DATA
        rospy.Subscriber("/SAR_Internal/camera/image_raw",Image,self.Camera_cb,queue_size = 1)
        rospy.Subscriber("/SAR_DC/StateData",SAR_StateData,self.SAR_StateDataCallback,queue_size = 1)

    def Create_csv(self):

        with open(self.Path,mode = 'w') as logfile:
            writer = csv.writer(logfile, delimiter = ',',quotechar = '"',quoting = csv.QUOTE_MINIMAL)
            writer.writerow(['Time','X_pos','Y_pos','Z_pos','X_vel','Y_vel','Z_vel','D_ceil','OFx','OFy','Tau','Camera_Data'])
            print(f'\nLogging file {self.Filename}.csv was successfully created\n')

    def Append_data(self):
        
        ## CLEAN CAMERA STRING
        Camera_data = np.array2string(self.Camera_raw,separator = ' ').replace('\n','').replace('[','').replace(']','') #throw raw camera array into a long string
        
        ## RENAME VARIABLES WITH CLEAR NAMES
        x_pos = self.posCF[0]
        y_pos = self.posCF[1]
        z_pos = self.posCF[2]

        x_vel = self.velCF[0]
        y_vel = self.velCF[1]
        z_vel = self.velCF[2]

        d_ceil = self.d_ceil
        Tau = self.Tau
        OFx = self.OFx
        OFy = self.OFy

        ## LOG IF WITHIN RANGE OF CEILING
        if(0.06 < d_ceil < 1.6): 

            with open(self.Path,mode = 'a') as logfile:
                writer = csv.writer(logfile, delimiter = ',', quotechar = '"', quoting = csv.QUOTE_MINIMAL)
                writer.writerow([self.t,x_pos,y_pos,z_pos,x_vel,y_vel,z_vel,d_ceil,OFx,OFy,Tau,Camera_data])


    def Camera_cb(self,Cam_msg):
        
        self.t = np.round(Cam_msg.header.stamp.to_sec(),4) # sim time
        self.Camera_raw = np.frombuffer(Cam_msg.data,np.uint8) # 1D array to package into CSV 
        
        ## In order to filter need to reshape and filter then go back to 1D
        
        self.Append_data()

    def SAR_StateDataCallback(self,StateData_msg):

        self.posCF = np.round([ StateData_msg.Pose.position.x,
                                StateData_msg.Pose.position.y,
                                StateData_msg.Pose.position.z],3)

        

        self.quatCF = np.round([StateData_msg.Pose.orientation.x,
                                StateData_msg.Pose.orientation.y,
                                StateData_msg.Pose.orientation.z,
                                StateData_msg.Pose.orientation.w],3)

        self.eulCF = np.round([ StateData_msg.Eul.x,
                                StateData_msg.Eul.y,
                                StateData_msg.Eul.z],3)

        ## CF_TWIST
        self.velCF = np.round([ StateData_msg.Twist.linear.x,
                                StateData_msg.Twist.linear.y,
                                StateData_msg.Twist.linear.z],3)

        self.omegaCF = np.round([StateData_msg.Twist.angular.x,
                                 StateData_msg.Twist.angular.y,
                                 StateData_msg.Twist.angular.z],3)

        ## CF_VISUAL STATES
        self.Tau = np.round(StateData_msg.Tau,3)
        self.OFx = np.round(StateData_msg.OFx,3)
        self.OFy = np.round(StateData_msg.OFy,3)
        self.d_ceil = np.round(StateData_msg.D_ceil,3)

        
    

if __name__ == '__main__':

    CameraParser() #initialize the class when run
    rospy.spin() #run until this program is shutdown