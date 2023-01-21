#!/usr/bin/env python3
import numpy as np

import os
import time
import sys
import subprocess
import rospy
import gym
from .CrazyflieEnv_Base import CrazyflieEnv_Base

## ROS MESSAGES AND SERVICES
from std_srvs.srv import Empty
from crazyflie_msgs.srv import domainRand,domainRandRequest


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
    def __init__(self):
        CrazyflieEnv_Base.__init__(self)
        self.env_name = "CF_Gazebo"

        ## GAZEBO SIMULATION INITIALIZATION            
        # self.gazeboTimeout = GZ_Timeout
        rospy.init_node("Crazyflie_Env_Sim_Node")

        ## LAUNCH GAZEBO
        self.launch_Gazebo() 
        rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
        
        ## LAUNCH CONTROLLER
        self.launch_Node_Controller()
        rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=5)

        ## LAUNCH CF_DC
        self.launch_Node_CF_DC()
        rospy.wait_for_service("/CF_DC/Cmd_CF_DC",timeout=5)

        print("[INITIATING] Gazebo simulation started")
    

    def sleep(self,time_s):
        """
        Sleep in terms of Gazebo sim seconds not real time seconds
        """

        t_start = self.t
        while self.t - t_start <= time_s:
            # print("Sleeping....")
            FailureModes = self.diagnosticTest()
            if any(FailureModes):
                self.Restart(FailureModes)
                self.done = True
                return False

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
        """ Launches Gazebo environment with crazyflie drone
        """        

        print("[STARTING] Starting Gazebo Process...")
        term_command = "rosnode kill /gazebo /gazebo_gui"
        os.system(term_command)
        time.sleep(1.0)

        term_command = "killall -9 gzserver gzclient"
        os.system(term_command)
        time.sleep(1.0)
        
        subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
            start_new_session=True, shell=True)

    def launch_Node_Controller(self):
        """ 
        Kill previous controller node if active and launch controller node
        """        
        
        print("[STARTING] Starting Controller Process...")
        os.system("rosnode kill /Controller_Node")
        time.sleep(0.5)
        subprocess.Popen( # Controller Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_control controller",
            close_fds=True, preexec_fn=os.setsid, shell=True)

    def launch_Node_CF_DC(self):
        """ 
        Kill previous CF_DC node if active and launch CF_DC node
        """        
        
        print("[STARTING] Starting CF_DC Process...")
        os.system("rosnode kill /CF_DataConverter_Node")
        time.sleep(0.5)
        subprocess.Popen( # CF_DC Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_data_converter CF_DataConverter",
            close_fds=True, preexec_fn=os.setsid, shell=True)

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

        if n_steps >= 50:
            while True:
                try:
                    rospy.wait_for_message('/clock', Clock, timeout=0.1)
                except:
                    break


        
    def callService(self,addr,srv,srv_type,retries=5):

        FailureModes = self.diagnosticTest()

        if any(FailureModes):
            self.Restart(FailureModes)
            self.done = True

        for retry in range(retries):

            try:
                # print(CYAN + f"[CALL_SERVICE] Calling: {addr}" + ENDC)
                rospy.wait_for_service(addr,timeout=1)
                service = rospy.ServiceProxy(addr, srv_type)
                service(srv)
                # print(CYAN + f"[CALL_SERVICE] Called: {addr}" + ENDC)

                return True

            except rospy.ServiceException as e:
                print(f"[WARNING] {addr} service call failed (callService)")
                print(f"[WARNING] {e}")
                FailureModes[2] = False

                self.Restart(FailureModes)
                self.done = True

            except rospy.exceptions.ROSException as e:
                print(f"[WARNING] {addr} service call failed (callService)")
                print(f"[WARNING] {e}")
                FailureModes[2] = False

                self.Restart(FailureModes)
                self.done = True
        
        return False

    def diagnosticTest(self):
        FailureModes = [False,False,False]
        sys.stdout.write(YELLOW)
        
        ## CHECK THAT GAZEBO IS FUNCTIONING
        try:
            # print(CYAN + f"[DIAGNOSTIC] Calling: /gazebo/pause_physics" + ENDC)
            rospy.wait_for_service("/gazebo/pause_physics",timeout=1)
            # print(CYAN + f"[DIAGNOSTIC] Called: /gazebo/pause_physics" + ENDC)

        except rospy.ROSException as e:
            print(f"[WARNING] /gazebo/pause_physics wait for service failed (callService)")
            print(f"[WARNING] {e}")
            FailureModes[0] = True

        ## CHECK THAT CONTROLLER IS FUNCTIONING
        try:
            # print(CYAN + f"[DIAGNOSTIC] Calling: /CTRL/Cmd_ctrl" + ENDC)
            rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=1)
            # print(CYAN + f"[DIAGNOSTIC] Called: /CTRL/Cmd_ctrl" + ENDC)
        except rospy.ROSException as e:
            print(f"[WARNING] /CTRL/Cmd_ctrl wait for service failed (callService)")
            print(f"[WARNING] {e}")
            FailureModes[1] = True

        ## CHECK THAT CF_DC IS FUNCTIONING
        try:
            # print(CYAN + f"[DIAGNOSTIC] Calling: /CF_DC/Cmd_CF_DC" + ENDC)
            rospy.wait_for_service('/CF_DC/Cmd_CF_DC',timeout=1)
            # print(CYAN + f"[DIAGNOSTIC] Called: /CF_DC/Cmd_CF_DC" + ENDC)

        except rospy.ROSException as e:
            print(f"[WARNING] /CF_DC/Cmd_CF_DC wait for service failed (callService)")
            print(f"[WARNING] {e}")
            FailureModes[2] = True

        sys.stdout.write(ENDC)
        return FailureModes

    def Restart(self,FailureModes=[True,False,False]):
        sys.stdout.write(YELLOW)

        if FailureModes == [True,False,False]:
            for retry in range(3):
                try:
                    print(f"[WARNING] Gazebo Restart. Attempt: {retry+1}/3 (restart_Gazebo)")
                    self.launch_Gazebo()
                    rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
                    print(GREEN+ f"[WARNING] Gazebo restart successful. (restart_Gazebo)" + ENDC)
                    return True

                except rospy.ROSException as e:
                    print(f"[WARNING] Gazebo restart Failed. (restart_Gazebo)")
                    print(f"[WARNING] {e}")
            
        sys.stdout.write(RED)
        for retry in range(5):
            try:
                print(f"[WARNING] Full Simulation Restart. Attempt: {retry+1}/5 (Restart Sim)")

                ## KILL EVERYTHING

                os.system("killall -9 gzserver gzclient")
                os.system("rosnode kill /gazebo /gazebo_gui /Controller_Node /CF_DataConverter_Node")
                time.sleep(2.0)
                
                subprocess.Popen( # Gazebo Process
                    "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
                    start_new_session=True, shell=True)
                rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
        
                ## LAUNCH CONTROLLER
                subprocess.Popen( # Controller Process
                    "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_control controller",
                    close_fds=True, preexec_fn=os.setsid, shell=True)
                rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=5)

                ## LAUNCH CF_DC
                subprocess.Popen( # CF_DC Process
                    "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_data_converter CF_DataConverter",
                    close_fds=True, preexec_fn=os.setsid, shell=True)
                rospy.wait_for_service("/CF_DC/Cmd_CF_DC",timeout=5)

                ## UNPAUSE GAZEBO
                rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
                print(GREEN + f"[WARNING] Gazebo restart successful. (Restart Sim)" + ENDC)

                return True

            except rospy.ROSException as e:
                print(f"[WARNING] Gazebo restart Failed. (Restart Sim)")
                print(f"[WARNING] {e}")

            print('\x1b[0m')

    def close(self):
        os.system("killall gzserver gzclient")

if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv_Sim()

    rospy.spin()