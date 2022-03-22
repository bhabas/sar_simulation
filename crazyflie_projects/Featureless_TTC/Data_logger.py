#!/usr/bin/env python3
import numpy as np
import getpass
import csv
import sys


#from pathlib import Path
from crazyflie_projects.Featureless_TTC.Camera_logger import CameraParser


class DataLogger:

    def __init__(self):

        #DEBUGGING#
        #time.sleep(2)
        #self.Filename = 'test.csv'

        self.Camera_parser = CameraParser() #initialize the class to access camera data

        self.Username = getpass.getuser()
        self.Path = f'/home/{self.Username}/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Featureless_TTC/logs' #store the logs in a folder for organization
        self.Filename = input("\ninput the name of the log file:\n") #wait for user input then create CSV with the given Filename
        self.Path = self.Path + '/' + self.Filename + '.csv'
        self.Create_csv()

        np.set_printoptions(threshold = sys.maxsize)
        self.t_init = self.Camera_parser.t #initialize the time collection
        self.Log_Flag = True
        self.logger()

    def Create_csv(self):

        with open(self.Path,mode = 'w') as logfile:
            writer = csv.writer(logfile, delimiter = ',',quotechar = '"',quoting = csv.QUOTE_MINIMAL)
            writer.writerow(['Camera_Data','Time','Z_pos','Z_vel'])
            print(f'\nLogging file {self.Filename}.csv was successfully created\n')

    def Append_data(self):
        
        Camera_data = np.array2string(self.Camera_parser.Camera_raw,separator = ' ').replace('\n','').replace('[','').replace(']','') #throw raw camera array into a long string
        time = np.round(self.Camera_parser.t,4) #round data to fit into pandas data frames
        z_pos = np.round(self.Camera_parser.z_pos,4)
        z_vel = np.round(self.Camera_parser.z_vel,4)
        
        with open(self.Path,mode = 'a') as logfile:
            writer = csv.writer(logfile, delimiter = ',', quotechar = '"', quoting = csv.QUOTE_MINIMAL)
            writer.writerow([Camera_data,time,z_pos,z_vel])
                
    def logger(self): #'rospy.spin()' for logging data until program is terminated

        t_prev = 0.

        while self.Log_Flag:

            if (self.Camera_parser.t != t_prev):
                #log data for 5 sim seconds
                if (self.Camera_parser.t - self.t_init <= 5):

                    self.Append_data()

                else:

                    self.Log_Flag = False
                    print('Flight log succesfully created!')

            t_prev = self.Camera_parser.t
            

if __name__=='__main__':

    Data_Logger = DataLogger() #init the class
    
    #remove and replace with while True loop that calls append data
    #rospy.spin() 
           