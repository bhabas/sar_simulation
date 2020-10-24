import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import socket, struct
from threading import Thread
import struct
from queue import Queue

import time
import os, subprocess, signal
import rospy
import csv

from socket import timeout

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class CrazyflieEnv:
    def __init__(self, port_RL, port_Ctrl,username):
        print("Init CrazyflieEnv")

        self.user = username   
        if self.user == 'bhabas':
            # import pyautogui    
            pass
        
        # Initializes the ROS node for the process. Can only have one nod in a rospy process
        rospy.init_node("crazyflie_env_node",anonymous=True) 
        self.launch_sim() 
    

        self.port_RL = port_RL # Reinforcment Learning Port
        self.port_Ctrl = port_Ctrl # Controller Port

        # fd is local server (Change name?)
        self.fd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # DGRAM socket doesn't care about lost data
        
        # set receive buffer size to be 112 bytes aka 14 doubles (1 double = 8 bytes)
        self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 112) # setsockopt = set socket options

        # set send buffer size to be 40 bytes. Both 112 bytes and 40 bytes are lower than 
        # the minimum value, so buffer size will be set as minimum value. See "'man 7 socket"
        self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 40)  


        self.addr_Ctrl = ("", self.port_Ctrl) # Controller Address
        self.addr_RL = ("", self.port_RL) # RL Address
        self.fd.bind(self.addr_RL) # bind() associates the socket with specific network interface and port number
        

        self.queue_command = Queue(3)
        self.path_all = np.zeros(shape=(14,8000,1))
        self.state_current = np.zeros(shape=(14))

        self.isRunning = True
        self.receiverThread = Thread(target=self.recvThread_Gazebo, args=()) # Thread for recvThread function
        self.receiverThread.daemon = True
        self.receiverThread.start() # Start recieverThread for recvThread function

        self.senderThread = Thread(target=self.sendThread_Ctrl, args=())
        self.senderThread.daemon = True

        

        # =========== Laser & Camera =========== #
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



    # ============================
    ##      Laser & Camera 
    # ============================
    
    def cam_callback(self,data): # callback function for camera subscriber
        self.camera_msg = data
        self.cv_image = self.bridge.imgmsg_to_cv2(self.camera_msg, desired_encoding='mono8')
        #self.camera_pixels = self.camera_msg.data

    def camThread(self): # Thread for recieving camera messages
        print('Start Camera Thread')
        self.camera_sub = rospy.Subscriber('/camera/image_raw',Image,self.cam_callback)
        rospy.spin()
    
    def scan_callback(self,data): # callback function for laser subsriber
        self.laser_msg = data
        if  self.laser_msg.ranges[0] == float('Inf'):
            # sets dist to 4 m if sensor reads Infinity (no walls)
            self.laser_dist = 4 # max sesnsor dist
        else:
            self.laser_dist = self.laser_msg.ranges[0]

    def lsrThread(self): # Thread for recieving laser scan messages
        print('Start Laser Scanner Thread')
        self.laser_sub = rospy.Subscriber('/zranger2/scan',LaserScan,self.scan_callback)
        rospy.spin()





    def close_sim(self):
        os.killpg(self.controller_p.pid, signal.SIGTERM)
        time.sleep(4)
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)
        time.sleep(4)  

    def delay_env_time(self,t_start,t_delay):
        # delay time defined in ms
        while (self.getTime() - t_start < t_delay/10000):
                    pass

    def __del__(self):
        self.isRunning = False
        self.fd.close()


    def enableSticky(self, enable): # enable=0 disable sticky, enable=1 enable sticky
        header = 10
        msg = struct.pack('5d', header, enable, 0, 0, 0)
        self.fd.sendto(msg, self.addr_Ctrl)
        time.sleep(0.001) # the sleep time after enableSticky(0) must be small s.t. the gazebo simulation is satble. Because the simulation after joint removed becomes unstable quickly.
    
    def getTime(self):
        return self.state_current[0]

    def launch_sim(self):
        ## There's some issue with the external shells that cause it to hang up on missed landings as it just sits on the ground


        self.gazebo_p = subprocess.Popen(
            "gnome-terminal --disable-factory -- ~/catkin_ws/src/crazyflie_simulation/src/4.\ rl/src/utility/launch_gazebo.bash", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
        time.sleep(5)

        self.controller_p = subprocess.Popen(
            "gnome-terminal --disable-factory --geometry 80x29 -- ~/catkin_ws/src/crazyflie_simulation/src/4.\ rl/src/utility/launch_controller.bash", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
        time.sleep(1)

    def pause(self): #Pause simulation
        os.system("rosservice call gazebo/pause_physics")
        return self.state_current

    def resume(self): #Resume simulation
        os.system("rosservice call gazebo/unpause_physics")
        return self.state_current


    def reset(self): # Disable sticky then places spawn_model at origin
        self.enableSticky(0)
        os.system("rosservice call gazebo/reset_world")
        self.enableSticky(0)
        time.sleep(0.1)
        return self.state_current
    
    def recvThread_Gazebo(self): # Recieve position data from Gazebo?
        print("Start recvThread from Gazebo")
        while self.isRunning:
            try:
                data, addr_remote = self.fd.recvfrom(112) # 1 double = 8 bytes
                x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z,sim_time = struct.unpack('14d',data) # unpack 112 byte msg into 14 doubles
                self.state_current = np.array([sim_time, x,y,z,qw,qx,qy,qz,vx,vy,vz,omega_x,omega_y,omega_z])
                self.timeout = False
            
            except timeout:
                self.timeout = True
            
            
    def sendThread_Ctrl(self):
        print("Start sendThread to Controller")
        while self.isRunning:
            buf = self.queue_command.get(block=True)
            len = self.fd.sendto(buf, self.addr_Ctrl)


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
        else:
            print("no such action")
        cmd = struct.pack('5d', header, ctrl_vals[0], ctrl_vals[1], ctrl_vals[2], ctrl_flag) # Send command
        self.fd.sendto(cmd, self.addr_Ctrl)
        time.sleep(0.1)
        cmd = struct.pack('5d',99,0,0,0,1) # Send blank command so controller doesn't keep redefining values
        self.fd.sendto(cmd, self.addr_Ctrl)

        #self.queue_command.put(buf, block=False)

        reward = 0
        done = 0
        info = 0
        return self.state_current, reward, done, info




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
                'mu','sigma',
                't','x','y','z',
                'qx','qy','qz','qw',
                'vx','vy','vz',
                'wx','wy','wz',
                'gamma','reward','reward_avg','n_rollouts'
                'RREV','omega_x','omega_u',"","","","","", # Place holders
                'error'])

    def IC_csv(self,agent,state,k_ep,k_run,reward=0,error="",v_ini = [0,0,0]):
        if self.record == True:
            with open(self.file_name, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    k_ep,k_run,
                    agent.alpha_mu.T,agent.alpha_sigma.T,
                    agent.mu.T,agent.sigma.T,
                    state[0],state[1],state[2],state[3], # t,x,y,z
                    state[4], state[5], state[6], state[7], # qx,qy,qz,qw
                    v_ini[1], v_ini[2],v_ini[0], # vx,vy,vz
                    state[11],state[12],state[13], # wx,wy,wz
                    agent.gamma,np.around(reward,2),"",agent.n_rollout,
                    "","","","","","","", # Place holders
                    error])


    def append_csv(self,agent,state,k_ep,k_run,sensor,reward=0,error=""):
        if self.record == True:
            state = np.around(state,3)
            sensor = np.around(sensor,3)

            with open(self.file_name, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    k_ep,k_run,
                    agent.alpha_mu.T,agent.alpha_sigma.T,
                    agent.mu.T,agent.sigma.T,
                    state[0],state[1],state[2],state[3], # t,x,y,z
                    state[4], state[5], state[6], state[7], # qx,qy,qz,qw
                    state[8], state[9],state[10], # vx,vy,vz
                    state[11],state[12],state[13], # wx,wy,wz
                    agent.gamma,np.around(reward,2),"",agent.n_rollout,
                    sensor[0],sensor[1],sensor[2],"","","","", # Place holders
                    error])


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