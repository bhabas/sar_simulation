#!/usr/bin/env python3
import numpy as np

import os
import time
import sys
import subprocess
from threading import Thread
import rospy
from crazyflie_env import SAR_Env_Base


class SAR_Env_Sim(SAR_Env_Base):

    def __init__(self,GZ_Timeout=True):
        SAR_Env_Base.__init__(self)

        self.gazebo_sim_process = None
        self.cf_dc_process = None
        self.controller_process = None
        

        ## KILL EVERYTHING
        os.system("killall -9 gzserver gzclient")
        os.system("rosnode kill /gazebo /gazebo_gui")
        time.sleep(1.0)
        os.system("rosnode kill /SAR_Controller_Node")
        time.sleep(1.0)
        os.system("rosnode kill /SAR_DataConverter_Node")
        time.sleep(1.0)

        self.start_all()
        self.start_monitoring()





        print("[INITIATING] Gazebo simulation started")

    
    def start_gazebo_sim(self):
        cmd = "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash"
        self.gazebo_sim_process = subprocess.Popen(cmd, shell=True)

    def start_cf_dc(self):
        cmd = "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun sar_data_converter SAR_DataConverter"
        self.cf_dc_process = subprocess.Popen(cmd, shell=True)

    def start_controller(self):
        cmd = "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun sar_control SAR_Controller"
        self.controller_process = subprocess.Popen(cmd, shell=True)

    def start_all(self):
        self.start_gazebo_sim()
        time.sleep(5)
        self.start_cf_dc()
        self.start_controller()

    def ping_subprocess(self, service_name):
        cmd = f"rosservice call {service_name}"
        try:
            result = subprocess.run(cmd, shell=True, timeout=5, check=True, stdout=subprocess.PIPE)
            return result.returncode == 0
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            return False

    def monitor_subprocesses(self):

        while True:

            gazebo_ping_ok = self.ping_subprocess("/gazebo/get_loggers")
            cf_dc_ping_ok = self.ping_subprocess("/SAR_DataConverter_Node/get_loggers")
            controller_ping_ok = self.ping_subprocess("/SAR_Controller_Node/get_loggers")

            if not (gazebo_ping_ok and cf_dc_ping_ok and controller_ping_ok):
                print("One or more subprocesses not responding. Restarting all subprocesses...")
                self.restart_all()

            time.sleep(1.0)

    def restart_all(self):
        os.system("killall -9 gzserver gzclient")
        os.system("rosnode kill /gazebo /gazebo_gui")
        time.sleep(1.0)
        os.system("rosnode kill /SAR_Controller_Node")
        time.sleep(1.0)
        os.system("rosnode kill /SAR_DataConverter_Node")
        time.sleep(1.0)

        self.start_all()

    def start_monitoring(self):
        monitor_thread = Thread(target=self.monitor_subprocesses)
        monitor_thread.daemon = True
        monitor_thread.start()


 

if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = SAR_Env_Sim()

    rospy.spin()


    