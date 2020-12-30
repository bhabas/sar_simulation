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
from crazyflie_rl.msg import RLData
from std_msgs.msg import Header 
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from cv_bridge import CvBridge

class CrazyflieEnv:
    def __init__(self,port_local=18050, port_Ctrl=18060):
        print("[STARTING] CrazyflieEnv is starting...")
        self.state_current = np.zeros(14)
        self.isRunning = True

        
        ## INIT ROS NODE FOR ENVIRONMENT 
        # NOTE: Can only have one node in a rospy process
        rospy.init_node("crazyflie_env_node") 
        self.launch_sim() 
    
        ## INIT RL SOCKET (AKA SERVER)
        self.RL_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Create UDP(DGRAM) Socket
        self.RL_port = port_local
        self.addr_RL = ("", self.RL_port) # Local address open to all incoming streams ("" = 0.0.0.0)
        self.RL_socket.bind(self.addr_RL) # Bind socket to specified port (0.0.0.0:18050)

        # Specify send/receive buffer sizes    
        self.RL_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 144) # State array from Ctrl [18 doubles]
        self.RL_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 40) # Action commands to Ctrl [5 doubles]
        self.RL_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
    
        ## INIT CONTROLLER ADDRESS ##
        self.port_Ctrl = port_Ctrl # Controller Server Port
        self.addr_Ctrl = ("", self.port_Ctrl) # Controller Address

        # kill -9 $(lsof -i:18050 -t)
        # if threads dont terminte when program is shut, you might need to use lsof to kill it

        
        


        self.k_run = 0
        self.k_ep = 0
        self.reward = 0
        self.reward_avg = 0
        self.n_rollouts = 0



  
        self.state_Sub = rospy.Subscriber('/global_state',GlobalState,self.global_stateCallback)
        self.laser_sub = rospy.Subscriber('/zranger2/scan',LaserScan,self.scan_callback)

        
        self.rewardThread = Thread(target=self.rewardPub,args=())
        self.rewardThread.start()
        

        self.timeoutThread = Thread(target=self.timeoutSub)
        self.timeoutThread.start()



        
        print("[COMPLETED] Environment done")



    

    # ============================
    ##   Publishers/Subscribers 
    # ============================


    def rewardPub(self):
        pub = rospy.Publisher('/rl_data',RLData,queue_size=10)
        r = rospy.Rate(50,reset=True)
        msg = RLData()
        header = Header()

        self.trial_name = ''

        self.k_run = 0
        self.k_ep = 0

        self.n_rollouts = 0
        self.gamma = 0
        self.logging_flag = False

        self.alpha_mu = []
        self.alpha_sigma = []
        self.mu = []
        self.sigma = []
        self.policy = []

        self.reward = 0

        while not rospy.is_shutdown():
            
            msg.header.stamp = rospy.Time.now()
            msg.k_ep = self.k_ep
            msg.k_run = self.k_run

            pub.publish(msg)
            r.sleep()


        
        
      

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
            pyautogui.moveTo(x=2500,y=0) 

            print("[STARTING] Starting Gazebo Process...")
            self.gazebo_p = subprocess.Popen( # Gazebo Process
                "gnome-terminal --disable-factory  -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility/launch_gazebo.bash", 
                close_fds=True, preexec_fn=os.setsid, shell=True)
            time.sleep(5)

            print("[STARTING] Starting Controller Process...")
            self.controller_p = subprocess.Popen( # Controller Process
                "gnome-terminal --disable-factory --geometry 81x33 -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility/launch_controller.bash", 
                close_fds=True, preexec_fn=os.setsid, shell=True)

        else:
            print("[STARTING] Starting Gazebo Process...")
            self.gazebo_p = subprocess.Popen( # Gazebo Process
                "gnome-terminal --disable-factory -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility/launch_gazebo.bash", 
                close_fds=True, preexec_fn=os.setsid, shell=True)
            time.sleep(5)

            print("[STARTING] Starting Controller Process...")
            self.controller_p = subprocess.Popen( # Controller Process
                "gnome-terminal --disable-factory --geometry 81x33 -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility/launch_controller.bash", 
                close_fds=True, preexec_fn=os.setsid, shell=True)

    def launch_dashboard(self):
        print("[STARTING] Starting Dashboard...")
        self.dashboard_p = subprocess.Popen(
            "gnome-terminal -- roslaunch dashboard_gui_pkg dashboard.launch",
            close_fds=True, preexec_fn=os.setsid, shell=True)




    def reset_pos(self): # Disable sticky then places spawn_model at origin
        self.enableSticky(0)

        state_msg = ModelState()
        state_msg.model_name = 'crazyflie_model'
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

            
    def step(self,action,ctrl_vals=[0,0,0],ctrl_flag=1): # Controller works to attain these values
        if action =='home': # default desired values/traj.
            header = 0
        elif action =='pos':  # position (x,y,z) 
            header = 1
        elif action =='vel':  # velocity (vx,vy,vz)
            header = 2
        elif action =='att':  # attitude: orientation (heading/yaw, pitch, roll/bank)
            header = 3
        elif action =='omega': # rotation rate (w_x:roll,w_y:pitch,w_z:yaw)
            header = 4
        elif action =='stop': # cut motors
            header = 5
        elif action =='gains': # assign new control gains
            header = 6
        else:
            header = 404
            print("no such action")
        cmd = struct.pack('5d', header, ctrl_vals[0], ctrl_vals[1], ctrl_vals[2], ctrl_flag) # Send command
        self.RL_socket.sendto(cmd, self.addr_Ctrl)
        time.sleep(0.01) # continually sends message during this time to ensure connection
        cmd = struct.pack('5d',8,0,0,0,1) # Send blank command so controller doesn't keep redefining values
        self.RL_socket.sendto(cmd, self.addr_Ctrl)

        #self.queue_command.put(buf, block=False)

        reward = 0
        done = 0
        info = 0
        return self.state_current, reward, done, info

    def cmd_send(self):
        while True:
            # Converts input number into action name
            cmd_dict = {0:'home',1:'pos',2:'vel',3:'att',4:'omega',5:'stop',6:'gains'}
            val = float(input("\nCmd Type (0:home,1:pos,2:vel,3:att,4:omega,5:stop,6:gains): "))
            action = cmd_dict[val]

            if action=='home' or action == 'stop': # Execute home or stop action
                ctrl_vals = [0,0,0]
                ctrl_flag = 1
                self.step(action,ctrl_vals,ctrl_flag)

            elif action=='gains': # Execture Gain changer
                
                vals = input("\nControl Gains (kp_x,kd_x,kp_R,kd_R): ") # Take in comma-seperated values and convert into list
                vals = [float(i) for i in vals.split(',')]
                ctrl_vals = vals[0:3]
                ctrl_flag = vals[3]

                self.step(action,ctrl_vals,ctrl_flag)
                
            elif action == 'omega': # Execture Angular rate action

                ctrl_vals = input("\nControl Vals (x,y,z): ")
                ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
                ctrl_flag = 1.0

                self.step('omega',ctrl_vals,ctrl_flag)


            else:
                ctrl_vals = input("\nControl Vals (x,y,z): ")
                ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
                ctrl_flag = float(input("\nController On/Off (1,0): "))
                self.step(action,ctrl_vals,ctrl_flag)

   

    # ============================
    ##      Timeout Functions 
    # ============================

    
    # Subscriber thread listens to /clock for any message
    def timeoutSub(self):
        rospy.Subscriber("/clock",Clock,self.timeoutCallback)
        self.timer = Timer(5,self.timeout)

    # If message is received reset the threading.Timer thread
    def timeoutCallback(self,msg):
        self.timer.cancel()
        self.timer = Timer(5,self.timeout)
        self.timer.start()
    
    # If no message in 5 seconds then close and relaunch sim
    def timeout(self):
        print("[RESTARTING] No Gazebo communication in 5 seconds")
        self.close_sim()
        time.sleep(1)
        self.launch_sim()