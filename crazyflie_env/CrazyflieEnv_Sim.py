#!/usr/bin/env python3
import numpy as np

import os
import time
import sys
import subprocess
import threading
import signal
import rospy
import gym
from .CrazyflieEnv_Base import CrazyflieEnv_Base

## ROS MESSAGES AND SERVICES
from std_srvs.srv import Empty
from crazyflie_msgs.srv import domainRand,domainRandRequest
from crazyflie_msgs.srv import ModelMove,ModelMoveRequest


from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty,EmptyRequest

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
CYAN = '\033[96m'
ENDC = '\033[m'

class CrazyflieEnv_Sim(CrazyflieEnv_Base,gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self,GZ_Timeout=True,debug_mode=False):
        CrazyflieEnv_Base.__init__(self)
        self.env_name = "CF_Gazebo"
        self.debug_mode = debug_mode

        ## GAZEBO SIMULATION INITIALIZATION            
        rospy.init_node("Crazyflie_Env_Sim_Node")

        ## KILL EVERYTHING
        os.system("killall -9 gzserver gzclient")
        os.system("rosnode kill /gazebo /gazebo_gui")
        time.sleep(2.0)
        os.system("rosnode kill /Controller_Node")
        time.sleep(2.0)
        os.system("rosnode kill /CF_DataConverter_Node")
        time.sleep(3.0)


        ## LAUNCH GAZEBO
        self.launch_Gazebo()

        ## LAUNCH CONTROLLER
        self.launch_Controller()

        ## LAUNCH CF_DC
        self.launch_CF_DC()



        ## START TIMEOUT/DIAGNOSTIC THREAD
        if GZ_Timeout == True:
            Timeout_Thread = threading.Thread(target=self.check_status)
            Timeout_Thread.name = "GZ_Timeout_Thread"
            Timeout_Thread.setDaemon(True)  
            Timeout_Thread.start()

        print("[INITIATING] Gazebo simulation started")

    
    def Set_Plane_Pose(self):

        ## CREATE SERVICE REQUEST MSG
        srv = ModelMoveRequest()
        srv.Model_Name = "Standard_Plane"
        srv.Pos_0.x = 1.5
        srv.Pos_0.y = 0
        srv.Pos_0.z = 1.5

        srv.Eul_0.x = 0
        srv.Eul_0.y = -90
        srv.Eul_0.z = 0

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/Landing_Surface_Pose',srv,ModelMove())

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
        self.callService('/gazebo/set_model_state',state_srv,SetModelState)
        self.iter_step(2)


        ## SET DESIRED VEL IN CONTROLLER
        self.SendCmd('GZ_traj',cmd_vals=[pos_0[0],vel_d[0],0],cmd_flag=0)
        self.iter_step(2)
        self.SendCmd('GZ_traj',cmd_vals=[pos_0[1],vel_d[1],0],cmd_flag=1)
        self.iter_step(2)
        self.SendCmd('GZ_traj',cmd_vals=[pos_0[2],vel_d[2],0],cmd_flag=2)
        self.iter_step(2)
        

    def reset_pos(self,z_0=0.358): # Disable sticky then places spawn_model at origin
        """Reset pose/twist of simulated drone back to home position. 
        As well as turning off stickyfeet

        Args:
            z_0 (float, optional): Starting height of crazyflie. Defaults to 0.379.
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

        os.system("roslaunch crazyflie_launch params.launch")
    

    # ============================
    ##      GAZEBO TECHNICAL
    # ============================
    def launch_Gazebo(self):
        """ Launches Gazebo environment with crazyflie quadrotor
        """        

        print("[STARTING] Starting Gazebo Process...")

        self.GZ_pro = subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
            start_new_session=True, shell=True)

        rospy.wait_for_service("/gazebo/pause_physics",timeout=30)

        self.Set_Plane_Pose()

    

    def launch_Controller(self):
        """ 
        Kill previous controller node if active and launch controller node
        """        

        print("[STARTING] Starting Controller Process...")
       
        self.ctrl_pro = subprocess.Popen( # Controller Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_control controller",
            close_fds=True, preexec_fn=os.setsid, shell=True)

        rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=20)

    

    def launch_CF_DC(self):
        """ 
        Kill previous CF_DC node if active and launch CF_DC node
        """        

        print("[STARTING] Starting CF_DC Process...")
        self.CF_DC_pro = subprocess.Popen( # CF_DC Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_data_converter CF_DataConverter",
            close_fds=True, preexec_fn=os.setsid, shell=True)

        rospy.wait_for_service("/CF_DC/Cmd_CF_DC",timeout=20)


    def callService(self,addr,srv,srv_type,retries=5):

        for retry in range(retries):

            try:
                rospy.wait_for_service(addr,timeout=1)
                service = rospy.ServiceProxy(addr, srv_type)
                service(srv)

                return True

            except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
                print(f"[WARNING] {addr} service call failed (callService)")
                print(f"[WARNING] {e}")

                self.done = True

        
        return False


    def check_status(self):
        """
        This function runs in a seperate thread that continually checks the status of the simulation.
        If connection to any of the Controller, CF_DC, and Gazebo fails then it will restart the full simulation.
        """        

        while True:

            if self.diagnosticTest() == False:

                print(f"{YELLOW}[WARNING] Diagnostic Test Failed. Restarting Simulation.{ENDC}")
                self.done = True

                for retry in range(3):

                    print(f"{YELLOW}[WARNING] Simulation Stalled. Restart Attempt: ({retry+1}/3).{ENDC}")

                    if self.Restart() == True:
                        if self.debug_mode:
                            print(f"{YELLOW}[WARNING] Simulation rebooted (diagnosticTest){ENDC}")
                        break

                    if retry == 2:
                        ## IF RESTART ATTEMPTS ARE EXCEEDED
                        print(f"{RED}[ERROR] Simulation Restart Failed. Closing Simulation. {ENDC}")
                        self.close()
                        break

            else:
                pass # DO NOTHING

                
                
            
            

    def diagnosticTest(self):
        
        ## CHECK THAT GAZEBO IS FUNCTIONING
        try:
            rospy.wait_for_message("/clock", Clock, timeout=10)
            if self.debug_mode:
                print(f"{YELLOW}[WARNING] /clock wait for message success (diagnosticTest){ENDC}")


        except rospy.ROSException as e:
            print(f"{YELLOW}[WARNING] /clock wait for message failed (diagnosticTest){ENDC}")
            print(f"{YELLOW}[WARNING] {e}{ENDC}")
            return False


        ## CHECK THAT CONTROLLER IS FUNCTIONING
        try:
            rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=1)
            if self.debug_mode:
                print(f"{YELLOW}[WARNING] /CF_DC/Cmd_ctrl wait for service success (diagnosticTest){ENDC}")


        except rospy.ROSException as e:
            print(f"{YELLOW}[WARNING] /CTRL/Cmd_ctrl wait for service failed (diagnosticTest){ENDC}")
            print(f"{YELLOW}[WARNING] {e}{ENDC}")
            return False

        ## CHECK THAT CF_DC IS FUNCTIONING
        try:
            rospy.wait_for_service('/CF_DC/Cmd_CF_DC',timeout=1)
            if self.debug_mode:
                print(f"{YELLOW}[WARNING] /CF_DC/Cmd_CF_DC wait for service success (diagnosticTest){ENDC}")


        except rospy.ROSException as e:
            print(f"{YELLOW}[WARNING] /CF_DC/Cmd_CF_DC wait for service failed (diagnosticTest){ENDC}")
            print(f"{YELLOW}[WARNING] {e}{ENDC}")
            return False

        return True

    def Restart(self):

        ## KILL EVERYTHING
        os.system("killall -9 gzserver gzclient")
        os.system("rosnode kill /gazebo /gazebo_gui")
        time.sleep(2.0)
        # os.system("rosnode kill /Controller_Node")
        # time.sleep(2.0)
        # os.system("rosnode kill /CF_DataConverter_Node")
        # time.sleep(3.0)

        
        ## LAUNCH GAZEBO
        try:
            os.killpg(os.getpgid(self.GZ_pro.pid), signal.SIGTERM)
            self.launch_Gazebo()

        except (rospy.ROSException,rospy.ROSInterruptException) as e:
            print(f"{YELLOW}[WARNING] Gazebo Launch Failed.{ENDC}")
            return False

        # ## LAUNCH CONTROLLER
        # try:
        #     os.killpg(os.getpgid(self.ctrl_pro.pid), signal.SIGTERM)
        #     self.launch_Controller()

        # except (rospy.ROSException,rospy.ROSInterruptException) as e:
        #     print(f"{YELLOW}[WARNING] Controller Launch Failed.{ENDC}")
        #     return False


        # ## LAUNCH CF_DC
        # try:
        #     os.killpg(os.getpgid(self.CF_DC_pro.pid), signal.SIGTERM)
        #     self.launch_CF_DC()

        # except (rospy.ROSException,rospy.ROSInterruptException) as e:
        #     print(f"{YELLOW}[WARNING] CF_DC Launch Failed.{ENDC}")
        #     return False


        return True

            

    def close(self):
        os.system("killall gzserver gzclient")




    def gazebo_pause_physics(self):
        srv = EmptyRequest()
        self.callService("/gazebo/pause_physics",srv,Empty)

    def gazebo_unpause_physics(self):
        srv = EmptyRequest()
        self.callService("/gazebo/unpause_physics",srv,Empty)
        
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


if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv_Sim()

    rospy.spin()