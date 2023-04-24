#!/usr/bin/env python3
import numpy as np

import os
import time
import sys
import subprocess
from threading import Thread,Event
import rospy
from crazyflie_env import SAR_Env_Base

from std_msgs.msg import Float64
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Clock


class SAR_Env_Sim(SAR_Env_Base):

    def __init__(self,GZ_Timeout=True):
        SAR_Env_Base.__init__(self)

        self.GZ_Sim_process = None
        self.SAR_DC_process = None
        self.SAR_Ctrl_process = None

        

        ## START SIMULATION
        self.Clock_Check_Flag = Event() # Stops clock monitoring during launch process
        self.restart_subprocesses()


        ## START MONITORING NODES
        self.start_monitoring_subprocesses()
        if GZ_Timeout == True:
            self.start_monitoring_clock_topic()





        print("[INITIATING] Gazebo simulation started")

    

    
    # ================================
    ##     SIM MONITORING/LAUNCH
    # ================================

    def launch_GZ_Sim(self):
        cmd = "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash"
        self.GZ_Sim_process = subprocess.Popen(cmd, shell=True)

    def launch_SAR_DC(self):
        cmd = "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun sar_data_converter SAR_DataConverter"
        self.SAR_DC_process = subprocess.Popen(cmd, shell=True)

    def launch_controller(self):
        cmd = "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun sar_control SAR_Controller"
        self.SAR_Ctrl_process = subprocess.Popen(cmd, shell=True)

    def start_monitoring_subprocesses(self):
        monitor_thread = Thread(target=self.monitor_subprocesses)
        monitor_thread.daemon = True
        monitor_thread.start()

    def monitor_subprocesses(self):

        while True:

            GZ_ping_ok = self.ping_subprocesses("/gazebo/get_loggers")
            SAR_DC_ping_ok = self.ping_subprocesses("/SAR_DataConverter_Node/get_loggers")
            SAR_Ctrl_ping_ok = self.ping_subprocesses("/SAR_Controller_Node/get_loggers")

            if not (GZ_ping_ok and SAR_DC_ping_ok and SAR_Ctrl_ping_ok):
                print("One or more subprocesses not responding. Restarting all subprocesses...")
                self.restart_subprocesses()

            time.sleep(0.5)

    def ping_subprocesses(self, service_name,silence_errors=False):
        cmd = f"rosservice call {service_name}"
        stderr_option = subprocess.DEVNULL if silence_errors else None

        try:
            result = subprocess.run(cmd, shell=True, timeout=5, check=True, stdout=subprocess.PIPE, stderr=stderr_option)
            return result.returncode == 0
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            return False
        
        
    def restart_subprocesses(self):

        self.Clock_Check_Flag.clear()

        ## KILL ALL POTENTIAL NODE/SUBPROCESSES
        os.system("killall -9 gzserver gzclient")
        os.system("rosnode kill /gazebo /gazebo_gui")
        time.sleep(2.0)
        os.system("rosnode kill /SAR_Controller_Node")
        time.sleep(2.0)
        os.system("rosnode kill /SAR_DataConverter_Node")
        time.sleep(2.0)

        ## LAUNCH GAZEBO
        self.launch_GZ_Sim()
        self.wait_for_gazebo_launch()

        self.launch_controller()
        self.launch_SAR_DC()

        self.Clock_Check_Flag.set()

    def wait_for_gazebo_launch(self):

        while not self.ping_subprocesses("/gazebo/get_loggers",silence_errors=True):

            print("Waiting for Gazebo to fully launch...")
            time.sleep(1)

        print("Gazebo has fully launched.")




    # ===========================
    ##      MONITOR CLOCK
    # ===========================

    def start_monitoring_clock_topic(self):
        monitor_clock_thread = Thread(target=self.monitor_clock_topic)
        monitor_clock_thread.daemon = True
        monitor_clock_thread.start()

    def monitor_clock_topic(self):

        while True:
            self.Clock_Check_Flag.wait()
            try:
                rospy.wait_for_message('/clock', Clock, timeout=5)
            except rospy.ROSException:
                print("No message received on /clock topic within the timeout. Unpausing physics.")
                self.pause_physics(False)

    def pause_physics(self,pause_flag=True):

        if pause_flag == True:
            service = '/gazebo/pause_physics'
        else:
            service = '/gazebo/unpause_physics'

        rospy.wait_for_service(service)
        try:
            service_call = rospy.ServiceProxy(service, Empty)
            service_call()
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")


 

if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = SAR_Env_Sim()

    rospy.spin()


    