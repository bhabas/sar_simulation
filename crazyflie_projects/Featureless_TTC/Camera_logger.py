#!/usr/bin/env python3
import rospy
import numpy as np
import getpass
import csv
import sys

from sensor_msgs.msg import Image
from crazyflie_msgs.msg import CF_StateData,CF_FlipData,CF_ImpactData,CF_MiscData
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
        self.RREV = 0.0
        self.d_ceil = 0.0 
        
        self.t = 0.0
        self.t_prev = 0.0
        self.n = 0

        self.Camera_raw = np.array([])

        #initialize params
        self.ceiling_h = rospy.get_param("/CEILING_HEIGHT")

        self.Username = getpass.getuser()
        self.Path = f'/home/{self.Username}/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Featureless_TTC/local_logs' #store the logs in a folder for organization
        self.Filename = input("\ninput the name of the log file:\n") #wait for user input then create CSV with the given Filename
        self.Path = self.Path + '/' + self.Filename + '.csv'
        self.Create_csv()
        np.set_printoptions(threshold = sys.maxsize) #allows it to print the full string without truncation

        #collect misc. data
        # while(self.Log_Flag):
        rospy.Subscriber("/CF_Internal/camera/image_raw",Image,self.Camera_cb,queue_size = 1)
        rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size = 1)

    def Create_csv(self):

        with open(self.Path,mode = 'w') as logfile:
            writer = csv.writer(logfile, delimiter = ',',quotechar = '"',quoting = csv.QUOTE_MINIMAL)
            writer.writerow(['Time','X_pos','Y_pos','Z_pos','X_vel','Y_vel','Z_vel','D_ceil','RREV','OFx','OFy','Tau','Camera_Data'])
            print(f'\nLogging file {self.Filename}.csv was successfully created\n')

    def Append_data(self):
        
        Camera_data = np.array2string(self.Camera_raw,separator = ' ').replace('\n','').replace('[','').replace(']','') #throw raw camera array into a long string
        time = np.round(self.t,6) #round data to fit into pandas data frames
        x_pos = np.round(self.posCF[0],4)
        y_pos = np.round(self.posCF[1],4)
        z_pos = np.round(self.posCF[2],4)

        x_vel = np.round(self.velCF[0],4)
        y_vel = np.round(self.velCF[1],4)
        z_vel = np.round(self.velCF[2],4)

        d_ceil = np.round(self.d_ceil,4)
        Tau = np.round(np.clip(self.Tau,-20,20),4)
        OFx = np.round(self.OFx,4)
        OFy = np.round(self.OFy,4)
        RREV = np.round(self.RREV,4)

        if(time != self.t_prev): #was getting duplicates

            if(d_ceil < 1.6 ): #prevent it from logging data when hovering 
                if(d_ceil > 0.06): #stop logging when contact occurs

                    with open(self.Path,mode = 'a') as logfile:
                        writer = csv.writer(logfile, delimiter = ',', quotechar = '"', quoting = csv.QUOTE_MINIMAL)
                        writer.writerow([time,x_pos,y_pos,z_pos,x_vel,y_vel,z_vel,d_ceil,RREV,OFx,OFy,Tau,Camera_data])

                else:
                    if(self.n < 1):
                        print('Flight log succesfully created!')
                        self.n += 1 #print once to let user know logging is finished

            self.t_prev = time

    def Camera_cb(self,Cam_msg):
        
        self.t = rospy.get_time() # sim time
        self.Camera_raw = np.frombuffer(Cam_msg.data,np.uint8) # 1D array to package into CSV
        self.Append_data()

    def CF_StateDataCallback(self,StateData_msg):

        self.t = np.round(StateData_msg.header.stamp.to_sec(),4)

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
        self.RREV = np.round(StateData_msg.RREV,3)
        self.d_ceil = np.round(StateData_msg.D_ceil,3)

        
    

if __name__ == '__main__':

    CameraParser() #initialize the class when run
    rospy.spin() #run until this program is shutdown