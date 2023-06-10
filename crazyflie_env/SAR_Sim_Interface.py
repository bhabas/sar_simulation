#!/usr/bin/env python3
import numpy as np

import os
import time
import subprocess
from threading import Thread,Event
import rospy
from crazyflie_env import SAR_Base_Interface


from std_srvs.srv import Empty
from rosgraph_msgs.msg import Clock
from crazyflie_msgs.srv import domainRand,domainRandRequest
from crazyflie_msgs.srv import ModelMove,ModelMoveRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
CYAN = '\033[96m'
ENDC = '\033[m'


class SAR_Sim_Interface(SAR_Base_Interface):

    def __init__(self,GZ_Timeout=True):
        SAR_Base_Interface.__init__(self)

        self.GZ_Sim_process = None
        self.SAR_DC_process = None
        self.SAR_Ctrl_process = None

        

        ## START SIMULATION
        self.Clock_Check_Flag = Event() # Stops clock monitoring during launch process
        self.restart_Sim()


        ## START MONITORING NODES
        self.start_monitoring_subprocesses()
        if GZ_Timeout == True:
            self.start_monitoring_clock_topic()





        print("[INITIATING] Gazebo simulation started")


    # =========================
    ##     ENV INTERACTION
    # =========================
    

    def iter_step(self,n_steps:int = 10):
        """Update simulation by n timesteps

        Args:
            n_steps (int, optional): Number of timesteps to step through. Defaults to 10.
        """        

        ## This might be better to be replaced with a world plugin with step function and a response when complete
        ## (https://github.com/bhairavmehta95/ros-gazebo-step)
        os.system(f'gz world --multi-step={int(n_steps)}')

        ## If num steps is large then wait until iteration is fully complete before proceeding
        if n_steps >= 50: 
            while True:
                try:
                    rospy.wait_for_message('/clock', Clock, timeout=0.1)
                except:
                    break

    def sleep(self,time_s):
        """
        Sleep in terms of Gazebo sim seconds not real time seconds
        """

        t_start = self.t
        while self.t - t_start <= time_s:

            ## NEGATIVE TIME DELTA
            if self.t < t_start:
                self.done = True
                return False

        return True
        
    def Vel_Launch(self,pos_0,vel_d,quat_0=[0,0,0,1]): 
        """Launch crazyflie from the specified position/orientation with an imparted velocity.
        NOTE: Due to controller dynamics, the actual velocity will NOT be exactly the desired velocity

        Args:
            pos_0 (list): Launch position [m]   | [x,y,z]
            vel_d (list): Launch velocity [m/s] | [Vx,Vy,Vz]
            quat_0 (list, optional): Orientation at launch. Defaults to [0,0,0,1].
        """        

        ## CREATE SERVICE MESSAGE
        state_srv = ModelState()
        state_srv.model_name = self.modelName

        ## INPUT POSITION AND ORIENTATION
        state_srv.pose.position.x = pos_0[0]
        state_srv.pose.position.y = pos_0[1]
        state_srv.pose.position.z = pos_0[2]

        state_srv.pose.orientation.x = quat_0[0]
        state_srv.pose.orientation.y = quat_0[1]
        state_srv.pose.orientation.z = quat_0[2]
        state_srv.pose.orientation.w = quat_0[3]

        ## INPUT LINEAR AND ANGULAR VELOCITY
        state_srv.twist.linear.x = vel_d[0]
        state_srv.twist.linear.y = vel_d[1]
        state_srv.twist.linear.z = vel_d[2]

        state_srv.twist.angular.x = 0
        state_srv.twist.angular.y = 0
        state_srv.twist.angular.z = 0

        ## PUBLISH MODEL STATE SERVICE REQUEST
        self.pause_physics()
        self.callService('/gazebo/set_model_state',state_srv,SetModelState)
        self.iter_step(2)


        ## SET DESIRED VEL IN CONTROLLER
        self.SendCmd('GZ_Const_Vel_Traj',cmd_vals=[pos_0[0],vel_d[0],0],cmd_flag=0)
        self.iter_step(2)
        self.SendCmd('GZ_Const_Vel_Traj',cmd_vals=[pos_0[1],vel_d[1],0],cmd_flag=1)
        self.iter_step(2)
        self.SendCmd('GZ_Const_Vel_Traj',cmd_vals=[pos_0[2],vel_d[2],0],cmd_flag=2)
        self.iter_step(2)
        

    def reset_pos(self,z_0=0.35): # Disable sticky then places spawn_model at origin
        """Reset pose/twist of simulated drone back to home position. 
        As well as turning off stickyfeet

        Args:
            z_0 (float, optional): Starting height of crazyflie. Defaults to 0.35.
        """        
        
        ## RESET POSITION AND VELOCITY
        state_srv = ModelState()
        state_srv.model_name = self.modelName
        state_srv.pose.position.x = 0.0
        state_srv.pose.position.y = 0.0
        state_srv.pose.position.z = z_0

        state_srv.pose.orientation.w = 1.0
        state_srv.pose.orientation.x = 0.0
        state_srv.pose.orientation.y = 0.0
        state_srv.pose.orientation.z = 0.0
        
        state_srv.twist.linear.x = 0.0
        state_srv.twist.linear.y = 0.0
        state_srv.twist.linear.z = 0.0

        state_srv.twist.angular.x = 0.0
        state_srv.twist.angular.y = 0.0
        state_srv.twist.angular.z = 0.0

        self.callService('/gazebo/set_model_state',state_srv,SetModelState)

    def updateInertia(self):

        ## CREATE SERVICE REQUEST MSG
        srv = domainRandRequest() 
        srv.mass = self.mass
        srv.Inertia.x = self.Ixx
        srv.Inertia.y = self.Iyy
        srv.Inertia.z = self.Izz

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/CF_Internal/DomainRand',srv,domainRand)

    def setParams(self):

        os.system("roslaunch sar_launch params.launch")

    

    
    # ================================
    ##     SIM MONITORING/LAUNCH
    # ================================

    def launch_GZ_Sim(self):
        cmd = "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun sar_launch launch_gazebo.bash"
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
            NaN_check_ok = not np.isnan(self.pos[0])

            if not (GZ_ping_ok and SAR_DC_ping_ok and SAR_Ctrl_ping_ok):
                print("One or more subprocesses not responding. Restarting all subprocesses...")
                self.restart_Sim()

            if not (NaN_check_ok):
                print("NaN value detected. Restarting all subprocesses...")
                self.restart_Sim()

            time.sleep(0.5)

    def ping_subprocesses(self, service_name,silence_errors=False):
        cmd = f"rosservice call {service_name}"
        stderr_option = subprocess.DEVNULL if silence_errors else None

        try:
            result = subprocess.run(cmd, shell=True, timeout=5, check=True, stdout=subprocess.PIPE, stderr=stderr_option)
            return result.returncode == 0
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            return False
        
    
    def restart_Sim(self):

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
        self.wait_for_gazebo_launch(timeout=10)

        self.launch_controller()
        self.launch_SAR_DC()

        self.Clock_Check_Flag.set()

    def wait_for_gazebo_launch(self,timeout=None):

        start_time = time.time()

        while not self.ping_subprocesses("/gazebo/get_loggers",silence_errors=True):

            if timeout is not None and time.time() - start_time > timeout:
                print("Timeout reached while waiting for Gazebo to launch.")
                return False
        
            print("Waiting for Gazebo to fully launch...")
            time.sleep(1)

        print("Gazebo has fully launched.")
        return True




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
                rospy.wait_for_message('/clock', Clock, timeout=10)
            except (rospy.ROSException, rospy.exceptions.ROSInterruptException):
                
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
    env = SAR_Sim_Interface()

    rospy.spin()


    