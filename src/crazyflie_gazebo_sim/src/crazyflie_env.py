import numpy as np
import pandas as pd
import pyautogui,getpass

import socket, struct
from threading import Thread
from multiprocessing import Process,Array,Value
import struct


import time
import os, subprocess, signal
import rospy
import csv

from socket import timeout

from sensor_msgs.msg import LaserScan, Image, Imu


from cv_bridge import CvBridge

class CrazyflieEnv:
    def __init__(self, port_local=18050, port_Ctrl=18060):
        print("[STARTING] CrazyflieEnv is starting...")
        self.timeout = False # Timeout flag for reciever thread
        self.state_current = np.zeros(18)
        self.isRunning = True
        
        ## INIT ROS NODE FOR THE PROCESS 
        # NOTE: Can only have one node in a rospy process
        rospy.init_node("crazyflie_env_node",anonymous=True) 
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

        ## START THREAD TO RECEIVE MESSAGES
        self.receiverThread = Thread(target=self.recvThread, args=()) # Thread for recvThread function
        self.receiverThread.daemon = True # Run thread as background process
        self.receiverThread.start() # Start recieverThread for recvThread function
        

        # self.fd.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEPORT,1)  
        ## BINDING ERROR: sudo netstat -nlp | grep 18060(Portnumber)
        ## sudo kill -9 [Process ID]
        # kill -9 $(lsof -i:18050 -t)


        
        self.imu_msg = Imu
        self.imu_thread = Thread(target=self.imuthreadfunc,args=())
        self.imu_thread.daemon = True
        self.imu_thread.start()
        
        # if threads dont terminte when program is shut, you might need to use lsof to kill it

        # =========== Sensors =========== #
        self.laser_msg = LaserScan # Laser Scan Message Variable
        self.laser_dist = 0
        # Start Laser data reciever thread
        self.laserThread = Thread(target = self.lsrThread, args=())
        self.laserThread.daemon=True
        self.laserThread.start()
        '''
        self.bridge = CvBridge() # Object to transform ros msg to cv image (np array)
        self.camera_msg = Image # Image Message Variable
        self.cv_image = np.array(0)
        # Start Camera data reciever thread
        self.cameraThread = Thread(target = self.camThread, args=())
        self.cameraThread.daemon=True
        self.cameraThread.start()'''

        #self.senderThread.start()
        print("[COMPLETED] Environment done")


    def close_sim(self):
            os.killpg(self.controller_p.pid, signal.SIGTERM)
            os.killpg(self.gazebo_p.pid, signal.SIGTERM)


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

    def get_state(self,STATE): # function for thread that will continually read current state
        while True:
            state = self.state_current
            qw = state[4]
            if qw==0: # Fix for zero-norm error during initialization where norm([qw,qx,qy,qz]=[0,0,0,0]) = undf
                state[4] = 1
            STATE[:] = state.tolist() # Save to global array for access across multi-processes

    def launch_sim(self):
       
        if getpass.getuser() == 'bhabas':
            pyautogui.moveTo(x=2500,y=0) 

        self.gazebo_p = subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/utility/launch_gazebo.bash", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
        time.sleep(5)

        self.controller_p = subprocess.Popen( # Controller Process
            "gnome-terminal --disable-factory --geometry 81x33 -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/utility/launch_controller.bash", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
        time.sleep(1)

    def pause(self): #Pause simulation
        os.system("rosservice call gazebo/pause_physics")
        return self.state_current

    def resume(self): #Resume simulation
        os.system("rosservice call gazebo/unpause_physics")
        return self.state_current


    def reset_pos(self): # Disable sticky then places spawn_model at origin
        self.enableSticky(0)
        os.system("rosservice call gazebo/reset_world")
        return self.state_current
    
    def recvThread(self):
        # Threaded function to continually read data from Controller Server
        print("[STARTING] recvThread is starting...")
        while self.isRunning:
            # Note: This doesn't want to receive data sometimes
            
            try:
                data, addr_remote = self.RL_socket.recvfrom(144) # 14d (1 double = 8 bytes)
                sim_time,x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,ms_1,ms_2,ms_3,m2_4 = struct.unpack('18d',data) # unpack 112 byte msg into 14 doubles
                self.state_current = np.array([sim_time, x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,ms_1,ms_2,ms_3,m2_4])
                # np.set_printoptions(precision=2, suppress=True)
                # print(self.state_current)
                self.timeout = False
            
            except timeout:
                self.timeout = True
            
            
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



    # ============================
    ##          Sensors
    # ============================
    def imuthreadfunc(self):
        self.imu_sub = rospy.Subscriber('/imu',Imu,self.imu_callback)
        rospy.spin()
    
    def imu_callback(self,data):
        self.imu_msg = data

    


    def cam_callback(self,data): # callback function for camera subscriber
        self.camera_msg = data
        self.cv_image = self.bridge.imgmsg_to_cv2(self.camera_msg, desired_encoding='mono8')
        #self.camera_pixels = self.camera_msg.data

    def camThread(self): # Thread for recieving camera messages
        print('[STARTING] Camera thread is starting...')
        self.camera_sub = rospy.Subscriber('/camera/image_raw',Image,self.cam_callback)
        rospy.spin()
    
    def lsrThread(self): # Thread for recieving laser scan messages
        print('[STARTING] Laser distance thread is starting...')
        self.laser_sub = rospy.Subscriber('/zranger2/scan',LaserScan,self.scan_callback)
        rospy.spin()

    def scan_callback(self,data): # callback function for laser subsriber
        self.laser_msg = data
        if  self.laser_msg.ranges[0] == float('Inf'):
            # sets dist to 4 m if sensor reads Infinity (no walls)
            self.laser_dist = 4 # max sesnsor dist
        else:
            self.laser_dist = self.laser_msg.ranges[0]

    

    # ============================
    ##       Data Recording 
    # ============================
    def create_csv(self,file_name,record=False):
        self.file_name = file_name
        self.record = record

        if self.record == True:
            with open(self.file_name, mode='w') as state_file:
                state_writer = csv.writer(state_file, delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                'k_ep','k_run',
                'alpha_mu','alpha_sig',
                'mu','sigma', 'policy',
                't','x','y','z',
                'qx','qy','qz','qw',
                'vx','vy','vz',
                'wx','wy','wz',
                'gamma','reward','reward_avg','n_rollouts',
                'RREV','OF_x','OF_y',"","","","", # Place holders
                'error'])

    def IC_csv(self,agent,state,k_ep,k_run,policy,v_d,omega_d,reward,error_str):
        if self.record == True:
            with open(self.file_name, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    k_ep,k_run,
                    agent.alpha_mu.T,agent.alpha_sigma.T,
                    agent.mu.T,agent.sigma.T,policy.T,
                    "","","","", # t,x,y,z
                    "", "", "", "", # qx,qy,qz,qw
                    v_d[0],v_d[1],v_d[2], # vx,vy,vz
                    omega_d[0],omega_d[1],omega_d[2], # wx,wy,wz
                    agent.gamma,np.around(reward,2),"",agent.n_rollout,
                    "","","", # Sensor readings
                    "","","","", # Place holders
                    error_str])


    def append_csv(self,agent,state,k_ep,k_run,sensor):
        if self.record == True:
            state = np.around(state,3)
            sensor = np.around(sensor,3)

            with open(self.file_name, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    k_ep,k_run,
                    "","", # alpha_mu,alpha_sig
                    "","","", # mu,sig,policy
                    state[0],state[1],state[2],state[3], # t,x,y,z
                    state[4], state[5], state[6], state[7], # qx,qy,qz,qw
                    state[8], state[9],state[10], # vx,vy,vz
                    state[11],state[12],state[13], # wx,wy,wz
                    "","","","", # gamma, reward, "", n_rollout
                    sensor[0],sensor[1],sensor[2],
                    "","","","", # Place holders
                    ""]) # error


    def append_csv_blank(self): 
        if self.record == True:
            with open(self.file_name, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([])

    def load_csv(self,datapath,k_ep):
        ## Load csv and seperate first run of the selected episode
        df = pd.read_csv(datapath)
        ep_data = df.loc[ (df['k_ep'] == k_ep) & (df['k_run'] == 0)]
        
        ## Create a list of the main values to be loaded
        vals = []
        for k in range(2,6):
            val = ep_data.iloc[0,k]
            val_edited = np.fromstring(val[2:-2], dtype=float, sep=' ')
            val_array = val_edited.reshape(1,-1).T
            vals.append(val_array)

        alpha_mu,alpha_sig, mu, sigma = vals

        return alpha_mu,alpha_sig,mu,sigma