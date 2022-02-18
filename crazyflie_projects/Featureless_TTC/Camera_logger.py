#!/usr/bin/env python3
import rospy
import numpy as np
import getpass
import csv
import sys

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


class CameraParser:


    def __init__(self):
        
        rospy.init_node('Camera_Data', anonymous = True)#start nodes

        #init variables
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0

        self.x_vel = 0.0
        self.y_vel = 0.0
        self.z_vel = 0.0

        self.t = 0.0
        self.t_prev = 0.0
        self.n = 0

        self.Camera_raw = np.array([])

        self.d_ceil =  0.0
        self.RREV = 0.0
        self.OFx = 0.0
        self.OFy = 0.0
        self.Tau = 0.0
        
        #initialize params
        self.ceiling_h = rospy.get_param("/CEILING_HEIGHT")

        self.Username = getpass.getuser()
        self.Path = f'/home/{self.Username}/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Featureless_TTC/logs' #store the logs in a folder for organization
        self.Filename = input("\ninput the name of the log file:\n") #wait for user input then create CSV with the given Filename
        self.Path = self.Path + '/' + self.Filename + '.csv'
        self.Create_csv()
        np.set_printoptions(threshold = sys.maxsize) #allows it to print the full string without truncation

        #collect misc. data
        # while(self.Log_Flag):
        self.Cam_sub = rospy.Subscriber("cf/camera/image_raw",Image,self.Camera_cb,queue_size = 1)
        self.Global_data_sub = rospy.Subscriber("env/global_state_data", Odometry,self.Global_State_cb,queue_size = 1)

    def Create_csv(self):

        with open(self.Path,mode = 'w') as logfile:
            writer = csv.writer(logfile, delimiter = ',',quotechar = '"',quoting = csv.QUOTE_MINIMAL)
            writer.writerow(['Time','X_pos','Y_pos','Z_pos','X_vel','Y_vel','Z_vel','D_ceil','RREV','OFx','OFy','Tau','Camera_Data'])
            print(f'\nLogging file {self.Filename}.csv was successfully created\n')

    def Append_data(self):
        
        Camera_data = np.array2string(self.Camera_raw,separator = ' ').replace('\n','').replace('[','').replace(']','') #throw raw camera array into a long string
        time = np.round(self.t,4) #round data to fit into pandas data frames
        x_pos = np.round(self.x_pos,4)
        y_pos = np.round(self.y_pos,4)
        z_pos = np.round(self.z_pos,4)

        x_vel = np.round(self.x_vel,4)
        y_vel = np.round(self.y_vel,4)
        z_vel = np.round(self.z_vel,4)

        d_ceil = np.round(self.d_ceil,4)
        RREV = np.round(self.RREV,4)
        OFx = np.round(self.OFx,4)
        OFy = np.round(self.OFy,4)
        Tau = np.round(np.clip(self.Tau,-20,20),4)

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
        
        self.t = np.round(rospy.get_time(), 4) #rounded sim time
        self.Camera_raw = np.frombuffer(Cam_msg.data,np.uint8) # 1D array to package into CSV
        self.Append_data()

    def Global_State_cb(self,msg):

        self.global_pos = msg.pose.pose.position
        self.x_pos = self.global_pos.x #x,y,z global positions
        self.y_pos = self.global_pos.y
        self.z_pos = self.global_pos.z

        self.global_vel = msg.twist.twist.linear
        self.x_vel = self.global_vel.x #x,y,z global velocities
        self.y_vel = self.global_vel.y
        self.z_vel = self.global_vel.z

        self.d_ceil = (self.ceiling_h - self.z_pos)
        self.RREV = self.z_vel/self.d_ceil
        self.OFx = -self.y_vel/self.d_ceil
        self.OFy = -self.x_vel/self.d_ceil
        self.Tau = self.d_ceil/self.z_vel

        
    

if __name__ == '__main__':

    CameraParser() #initialize the class when run
    rospy.spin() #run until this program is shutdown