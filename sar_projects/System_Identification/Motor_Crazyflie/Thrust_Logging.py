#!/usr/bin/env python3

import time
import threading
import numpy as np
import rospy
import csv

import sys
import rospkg
import os

BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_msgs'))
sys.path.insert(1,BASE_PATH)

from sar_env import CrazyflieEnv
from sar_msgs.msg import MS
from sar_msgs.msg import RLCmd



if __name__ == '__main__':

    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv()

    filePath = f"thrust.csv"
    with open(filePath,mode='w') as data_file:
        data_writer = csv.writer(data_file,delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
        data_writer.writerow(['t','thrust_input','thrust_actual'])




    def logging_Callback(msg):

        if msg.Motor_Number == 4:
            with open(filePath,mode='a') as data_file:
                data_writer = csv.writer(data_file,delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
                data_writer.writerow([np.round(env.t,3),np.round(msg.MotorThrust,3),np.round(msg.MotorThrust_actual,3)])


    rospy.Subscriber("/SAR_Internal/MS",MS,logging_Callback,queue_size=1)
    rospy.Publisher('/RL/cmd',RLCmd,queue_size=10)
    input("Start")

    t_start = env.t
    thrust_d = 0
    OnceFlag1 = True
    OnceFlag2 = True
    OnceFlag3 = True
    OnceFlag4 = True
    OnceFlag5 = True


    while True:
        t = env.t - t_start

        if t>=0 and OnceFlag1:
            OnceFlag1 = False
            thrust_d = 0.0
            env.sendCmd('thrusts',[0,0,0],thrust_d)
            print(f"Thrust[g] Cmd: {thrust_d:.3f}")


        if t >= 5 and OnceFlag2:
            OnceFlag2 = False
            thrust_d = 8.14
            env.sendCmd('thrusts',[0,0,0],thrust_d)
            print(f"Thrust[g] Cmd: {thrust_d:.3f}")

        if t >= 10 and OnceFlag3:
            OnceFlag3 = False
            thrust_d = 14.3
            env.sendCmd('thrusts',[0,0,0],thrust_d)
            print(f"Thrust[g] Cmd: {thrust_d:.3f}")

        if t >= 15 and OnceFlag4:
            OnceFlag4 = False
            thrust_d = 5.3
            env.sendCmd('thrusts',[0,0,0],thrust_d)
            print(f"Thrust[g] Cmd: {thrust_d:.3f}")

        if t >= 20 and OnceFlag5:
            OnceFlag5 = False
            thrust_d = 12.0
            env.sendCmd('thrusts',[0,0,0],thrust_d)
            print(f"Thrust[g] Cmd: {thrust_d:.3f}")

        if t>=25:
            env.sendCmd('thrusts',[0,0,0],0)
            break

    input("End Here")

