#!/usr/bin/env python3

import time
import threading
import numpy as np
import rospy
import csv

import sys
import rospkg
import os

BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_msgs'))
sys.path.insert(1,BASE_PATH)

from Crazyflie_env import CrazyflieEnv
from crazyflie_msgs.msg import MS



if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv()

    filePath = f"thrust.csv"
    with open(filePath,mode='w') as data_file:
        data_writer = csv.writer(data_file,delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
        data_writer.writerow(['t','thrust_input','thrust_actual'])




    def logging_Callback(msg):

        if msg.Motor_Number == 1:
            with open(filePath,mode='a') as data_file:
                data_writer = csv.writer(data_file,delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
                data_writer.writerow([np.round(env.t,3),np.round(msg.MotorThrust,3),np.round(msg.MotorThrust_actual,3)])


    rospy.Subscriber("/CF_Internal/MS",MS,logging_Callback,queue_size=1)


    rospy.spin()
