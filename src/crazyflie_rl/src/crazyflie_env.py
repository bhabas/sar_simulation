import numpy as np
import pandas as pd
import pyautogui,getpass

import socket, struct
from threading import Thread, Timer
import struct


import time
import os, subprocess, signal
import rospy
import csv


from sensor_msgs.msg import LaserScan, Image, Imu
from gazebo_communication_pkg.msg import GlobalState 
from crazyflie_rl.msg import RLData,RLCmd
from std_msgs.msg import Header 
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from cv_bridge import CvBridge

class CrazyflieEnv:
    def __init__(self):
        print("[STARTING] CrazyflieEnv is starting...")
        self.state_current = np.zeros(14)
        self.isRunning = True

        
        ## INIT ROS NODE FOR ENVIRONMENT 
        # NOTE: Can only have one node in a rospy process
        rospy.init_node("crazyflie_env_node") 
        self.launch_sim() 
    

        

  
        self.state_Subscriber = rospy.Subscriber('/global_state',GlobalState,self.global_stateCallback)
        self.laser_Subscriber = rospy.Subscriber('/zranger2/scan',LaserScan,self.scan_callback)
        self.RL_Publisher = rospy.Publisher('/rl_data',RLData,queue_size=10)
        self.Cmd_Publisher = rospy.Publisher('/rl_ctrl',RLCmd,queue_size=10)




        self.trial_name = ''
        self.agent = ''

        self.n_rollouts = 0
        self.gamma = 0
        self.h_ceiling = 0

        self.flip_flag = False
        self.runComplete_flag = False
        self.logging_flag = False
        self.createCSV_flag = False

        self.k_run = 0
        self.k_ep = 0

        self.alpha_mu = []
        self.alpha_sigma = []
        self.mu = []
        self.sigma = []
        self.policy = []

        self.reward = 0

        self.omega_d = [0,0,0]
        self.vel_d = [0,0,0]

        
        
        

        self.timeoutThread = Thread(target=self.timeoutSub)
        self.timeoutThread.start()



        
        print("[COMPLETED] Environment done")



    

    # ============================
    ##   Publishers/Subscribers 
    # ============================


    def RL_Publish(self):

        msg = RLData()
        header = Header()

        msg.header.stamp = rospy.Time.now()
        msg.trial_name = self.trial_name
        msg.agent = self.agent

        msg.logging_flag = self.logging_flag
        msg.createCSV_flag = self.createCSV_flag
        msg.flip_flag = self.flip_flag
        msg.runComplete_flag = self.runComplete_flag

        msg.n_rollouts = self.n_rollouts
        msg.gamma = self.gamma
        msg.h_ceiling = self.h_ceiling


        msg.k_ep = self.k_ep
        msg.k_run = self.k_run

        msg.alpha_mu = self.alpha_mu
        msg.alpha_sigma = self.alpha_sigma

        

        msg.mu = self.mu
        msg.sigma = self.sigma
        msg.policy = self.policy

        msg.reward = self.reward

        msg.vel_d = self.vel_d
        msg.omega_d = self.omega_d

        self.RL_Publisher.publish(msg)
     


        
        
      

    def global_stateCallback(self,msg):
        gs_msg = msg # gs_msg <= global_state_msg

        ## SET TIME VALUE FROM TOPIC
        t_temp = gs_msg.header.stamp.secs
        ns_temp = gs_msg.header.stamp.nsecs
        t = t_temp+ns_temp*1e-9 # (seconds + nanoseconds)
        
        ## SIMPLIFY STATE VALUES FROM TOPIC
        global_pos = gs_msg.global_pose.position
        global_quat = gs_msg.global_pose.orientation
        global_vel = gs_msg.global_twist.linear
        global_omega = gs_msg.global_twist.angular
        
        if global_quat.w == 0: # If zero at startup set quat.w to one to prevent errors
            global_quat.w = 1

        ## SET STATE VALUES FROM TOPIC
        position = [global_pos.x,global_pos.y,global_pos.z]
        orientation_q = [global_quat.w,global_quat.x,global_quat.y,global_quat.z]
        velocity = [global_vel.x,global_vel.y,global_vel.z]
        omega = [global_omega.x,global_omega.y,global_omega.z]


        ## COMBINE INTO COMPREHENSIVE LIST
        self.state_current = [t] + position + orientation_q +velocity + omega ## t (float) -> [t] (list)

    

        
        
        
        

    
    # ============================
    ##    Sensors/Gazebo Topics
    # ============================



    def scan_callback(self,data): # callback function for laser subsriber
        self.laser_msg = data
        if  self.laser_msg.ranges[0] == float('Inf'):
            # sets dist to 4 m if sensor reads Infinity (no walls)
            self.laser_dist = 4 # max sesnsor dist
        else:
            self.laser_dist = self.laser_msg.ranges[0]




    # ============================
    ##       Sim Operation
    # ============================

    def relaunch_sim(self):
        self.close_sim()
        time.sleep(1)
        self.launch_sim()

    def close_sim(self):
        os.killpg(self.controller_p.pid, signal.SIGTERM)
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)

    def close_dashboard(self):
        os.killpg(self.dashboard_p.pid, signal.SIGTERM)


    def delay_env_time(self,t_start,t_delay):
        # delay time defined in ms
        while (self.getTime() - t_start < t_delay/10000):
                    pass

    def __del__(self):
        self.isRunning = False
        self.RL_socket.close()


    def enableSticky(self, enable): # enable=0 disable sticky, enable=1 enable sticky
        header = 11
        msg = struct.pack('5d', header, enable, 0, 0, 0)
        self.RL_socket.sendto(msg, self.addr_Ctrl)
        time.sleep(0.001) # This ensures the message is sent enough times Gazebo catches it or something
        # the sleep time after enableSticky(0) must be small s.t. the gazebo simulation is satble. Because the simulation after joint removed becomes unstable quickly.

    def getTime(self):
        return self.state_current[0]

    def launch_sim(self):
       
        if getpass.getuser() == 'bhabas':
            

            print("[STARTING] Starting Gazebo Process...")
            self.gazebo_p = subprocess.Popen( # Gazebo Process
                "gnome-terminal --disable-factory  -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility/launch_gazebo.bash", 
                close_fds=True, preexec_fn=os.setsid, shell=True)
            time.sleep(5)

            print("[STARTING] Starting Controller Process...")
            self.controller_p = subprocess.Popen( # Controller Process
                "gnome-terminal --disable-factory --geometry 70x36 -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility/launch_controller.bash", 
                close_fds=True, preexec_fn=os.setsid, shell=True)

        else:
            print("[STARTING] Starting Gazebo Process...")
            self.gazebo_p = subprocess.Popen( # Gazebo Process
                "gnome-terminal --disable-factory -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility/launch_gazebo.bash", 
                close_fds=True, preexec_fn=os.setsid, shell=True)
            time.sleep(5)

            print("[STARTING] Starting Controller Process...")
            self.controller_p = subprocess.Popen( # Controller Process
                "gnome-terminal --disable-factory --geometry 70x36 -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility/launch_controller.bash", 
                close_fds=True, preexec_fn=os.setsid, shell=True)



    def launch_dashboard(self):
        print("[STARTING] Starting Dashboard...")
        self.dashboard_p = subprocess.Popen(
            "gnome-terminal -- roslaunch dashboard_gui_pkg dashboard.launch",
            close_fds=True, preexec_fn=os.setsid, shell=True)




    def reset_pos(self): # Disable sticky then places spawn_model at origin
        
        self.step('sticky',ctrl_flag=0)

        state_msg = ModelState()
        state_msg.model_name = 'crazyflie_model_X'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.0

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1

        state_msg.twist.linear.x = 0
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state_srv(state_msg)

    def step(self,action,ctrl_vals=[0,0,0],ctrl_flag=1):
        cmd_msg = RLCmd()

        cmd_dict = {'home':0,
                    'pos':1,
                    'vel':2,
                    'att':3,
                    'omega':4,
                    'stop':5,
                    'gains':6,
                    'moments':7,
                    'sticky':11}
        

        cmd_msg.cmd_type = cmd_dict[action]
        cmd_msg.cmd_vals.x = ctrl_vals[0]
        cmd_msg.cmd_vals.y = ctrl_vals[1]
        cmd_msg.cmd_vals.z = ctrl_vals[2]
        cmd_msg.cmd_flag = ctrl_flag
        
        self.Cmd_Publisher.publish(cmd_msg)


    def cmd_send(self):
        while True:
            # Converts input number into action name
            cmd_dict = {0:'home',1:'pos',2:'vel',3:'att',4:'omega',5:'stop',6:'gains'}
            val = float(input("\nCmd Type (0:home,1:pos,2:vel,3:att,4:omega,5:stop,6:gains): "))
            action = cmd_dict[val]

            if action=='home' or action == 'stop': # Execute home or stop action
                ctrl_vals = [0,0,0]
                ctrl_flag = 1
                self.stepPub(action,ctrl_vals,ctrl_flag)

            elif action=='gains': # Execture Gain changer
                
                vals = input("\nControl Gains (kp_x,kd_x,kp_R,kd_R): ") # Take in comma-seperated values and convert into list
                vals = [float(i) for i in vals.split(',')]
                ctrl_vals = vals[0:3]
                ctrl_flag = vals[3]

                self.stepPub(action,ctrl_vals,ctrl_flag)
                
            elif action == 'omega': # Execture Angular rate action

                ctrl_vals = input("\nControl Vals (x,y,z): ")
                ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
                ctrl_flag = 1.0

                self.stepPub('omega',ctrl_vals,ctrl_flag)


            else:
                ctrl_vals = input("\nControl Vals (x,y,z): ")
                ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
                ctrl_flag = float(input("\nController On/Off (1,0): "))
                self.stepPub(action,ctrl_vals,ctrl_flag)

   

    # ============================
    ##      Timeout Functions 
    # ============================

    # Subscriber thread listens to /clock for any message
    def timeoutSub(self):
        ## START INITIAL TIMERS
        self.timer_unpause = Timer(5,self.timeout_unpause)
        self.timer_relaunch = Timer(10,self.timeout_relaunch)
        ## START ROS CREATED THREAD FOR SUBSCRIBER
        rospy.Subscriber("/clock",Clock,self.timeoutCallback)
        ## END FUNCTION, THIS MIGHT NOT NEED TO BE THREADED?

    
    # If message is received reset the threading.Timer threads
    def timeoutCallback(self,msg):
        ## RESET TIMER THAT ATTEMPTS TO UNPAUSE SIM
        self.timer_unpause.cancel()
        self.timer_unpause = Timer(5,self.timeout_unpause)
        self.timer_unpause.start()

        ## RESET TIMER THAT RELAUNCHES SIM
        self.timer_relaunch.cancel()
        self.timer_relaunch = Timer(10,self.timeout_relaunch)
        self.timer_relaunch.start()
    

    def timeout_unpause(self):
        print("[UNPAUSING] No Gazebo communication in 5 seconds")
        os.system("rosservice call gazebo/unpause_physics")

    def timeout_relaunch(self):
        print("[RELAUNCHING] No Gazebo communication in 10 seconds")
        self.close_sim()
        time.sleep(1)
        self.launch_sim()