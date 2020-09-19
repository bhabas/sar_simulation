import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import socket, struct
from threading import Thread
import struct
from queue import Queue

import time
import os, subprocess, signal, shlex
import rospy
import csv



class CrazyflieEnv:
    def __init__(self, port_self, port_remote):
        print("Init CrazyflieEnv")
        rospy.init_node("crazyflie_env_node",anonymous=True)
        self.launch_sim()
        

        self.port_self = port_self
        self.port_remote = port_remote

        self.fd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 112)       # set receive buffer size to be 112 bytes (1 double = 8 bytes)
        self.fd.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 40)        # set send buffer size to be 40 bytes. Both 112 bytes and 40 bytes are lower than the minimum value, so buffer size will be set as minimum value. See "'man 7 socket"
        self.fd.bind( ("", self.port_self) )
        self.addr_remote_send = ("", self.port_remote)
        buf = struct.pack('5d', 2, 0, 0, 0, 0)
        self.fd.sendto(buf, self.addr_remote_send)

        self.queue_command = Queue(3)
        self.path_all = np.zeros(shape=(14,8000,1))
        self.state_current = np.zeros(shape=(14))
        self.logDataFlag = False

        self.isRunning = True
        self.receiverThread = Thread(target=self.recvThread, args=())
        self.receiverThread.daemon = True
        self.receiverThread.start()
        self.senderThread = Thread(target=self.sendThread, args=())
        self.senderThread.daemon = True

        #self.senderThread.start()
    
    def close_sim(self):
        os.killpg(self.controller_p.pid, signal.SIGTERM)
        time.sleep(5)
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)
        time.sleep(5)  

    def delay_env_time(self,t_start,t_delay):
        # delay time defined in ms
        while (self.getTime() - t_start < t_delay/10000):
                    pass

    def __del__(self):
        self.isRunning = False
        self.fd.close()


    def enableSticky(self, enable):      # enable=0 disable sticky, enable=1 enable sticky
        header = 11
        buf = struct.pack('5d', header, enable, 0, 0, 0)
        self.fd.sendto(buf, self.addr_remote_send)
        time.sleep(0.001)               # the sleep time after enableSticky(0) must be small s.t. the gazebo simulation is satble. Because the simulation after joint removed becomes unstable quickly.
    
    def getTime(self):
        return self.state_current[0]

    def launch_sim(self):
        ## There's some issue with the external shells that cause it to hang up on missed landings as it just sits on the ground
        # self.gazebo_p_ = subprocess.Popen(shlex.split(
        #     'gnome-terminal -x bash -c "~/catkin_ws/src/crazyflie_simulation/src/4.\ rl/src/launch_gazebo.bash"'),
        #     close_fds=True, preexec_fn=os.setsid)
        #     ### wait till open?
        # time.sleep(5)
        
        # self.controller_p_ = subprocess.Popen(shlex.split(
        #     'gnome-terminal --tab -x bash -c "~/catkin_ws/src/crazyflie_simulation/src/4.\ rl/src/launch_controller.bash"'),
        #     close_fds=True, preexec_fn=os.setsid)
        # time.sleep(5)     

        wd = os.getcwd()
        self.gazebo_p = subprocess.Popen(
            "gnome-terminal --disable-factory -- src/4.\ rl/src/launch_gazebo.bash", 
            close_fds=True, preexec_fn=os.setsid, shell=True,cwd=wd)
        time.sleep(5)
        self.controller_p = subprocess.Popen(
            "gnome-terminal --disable-factory -- src/4.\ rl/src/launch_controller.bash", 
            close_fds=True, preexec_fn=os.setsid, shell=True,cwd=wd)
        time.sleep(5)

    def pause(self): #Pause simulation
        os.system("rosservice call gazebo/pause_physics")
        return self.state_current

    def resume(self): #Resume simulation
        os.system("rosservice call gazebo/unpause_physics")
        return self.state_current


    def reset(self): #Spends 2 seconds resetting the world and 3 seconds waiting after that
        self.enableSticky(0)
        os.system("rosservice call gazebo/reset_world")
        time.sleep(1)
        return self.state_current
    
    def recvThread(self):
        print("Start recvThread")
        path = None
        
        k_run = 0
        while self.isRunning:
            k_run = k_run + 1

            data, addr_remote = self.fd.recvfrom(112)     # 1 double = 8 bytes
            px,py,pz,q0,q1,q2,q3,vx,vy,vz,p,q,r,sim_time = struct.unpack('14d',data)
            self.state_current = np.array([sim_time, px,py,pz,q0,q1,q2,q3,vx,vy,vz,p,q,r])

            if any( np.isnan( self.state_current ) ):
                self.logDataFlag = False
                path = None
            
            if self.logDataFlag:
                if path is None:
                    path = np.zeros(shape=(14,8000,1))
                    k = 0
                if k%50==0:
                    path[:,int(k/50),0] = np.array([sim_time,px,py,pz,q0,q1,q2,q3,vx,vy,vz,p,q,r])
                    #path[:,k,0] = np.array([[sim_time],[px],[py],[pz],[q0],[q1],[q2],[q3],[vx],[vy],[vz],[p],[q],[r]])
                k = k + 1
            
            if not self.logDataFlag and path is not None:
                self.path_all = np.append(self.path_all, path, axis=2)
                path = None
                
            '''if k_run%50 == 1:
                # print("received data ", data, " from ", addr_remote)
                print("=============================================================")
                print( 'Position: px=%.3f, py=%.3f, pz=%.3f' %(px, py, pz) )
                print( 'Orientat: q0=%.3f, q1=%.3f, q2=%.3f q3=%.3f' %(q0, q1, q2, q3) )
                print( 'Velocity: vx=%.3f, vy=%.3f, vz=%.3f' %(vx, vy, vz) )
                print( 'AnguRate: p =%.3f, q =%.3f, r =%.3f' %(p, q, r) )
                print( 'SimTime : time =%.3f' %(sim_time) )'''
            
            
    def sendThread(self):
        print("Start sendThread")
        while self.isRunning:
            #time.sleep(0.5)
            #buf = struct.pack('5d', 2.0, 1.1, 2.2, 3.3, 4.4)
            buf = self.queue_command.get(block=True)
            len = self.fd.sendto(buf, self.addr_remote_send)
            '''if len>0:
                print("Send to controller succeed!")
            else:
                print("Send to controller failed!")'''

   

    def step(self, action):
        if action['type'] == 'pos':
            header = 1
        elif action['type'] == 'vel':
            header = 2
        elif action['type'] == 'att':
            header = 3
        elif action['type'] == 'rate':
            header = 4
        else:
            print("no such action")
        x = action['x']
        y = action['y']
        z = action['z']
        additional = action['additional']

        buf = struct.pack('5d', header, x, y, z, additional)
        self.fd.sendto(buf, self.addr_remote_send)
        #self.queue_command.put(buf, block=False)

        reward = 0
        done = 0
        info = 0
        return self.state_current, reward, done, info




    # ============================
    ##       Data Recording 
    # ============================
    def create_csv(self,file_name):
        self.file_name = file_name
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
            'gamma','reward','reward_avg',
            "","","","","","","","", # Place holders
            'error'])


    def append_csv(self,agent,state,k_ep,k_run,reward=0,error=""): 
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
                agent.gamma,np.around(reward,2),"",
                "","","","","","","","", # Place holders
                error])


    def append_csv_blank(self): 
        with open(self.file_name, mode='a') as state_file:
            state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            state_writer.writerow([])