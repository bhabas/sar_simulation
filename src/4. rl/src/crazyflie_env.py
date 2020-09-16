import numpy as np
import socket
from threading import Thread
import struct
from queue import Queue
import matplotlib.pyplot as plt
import time
import os, subprocess, signal
import rospy
import shlex


class CrazyflieEnv:
    def __init__(self, port_self, port_remote):
        print("Init CrazyflieEnv")
        rospy.init_node("crazyflie_env_node",anonymous=True)
        self.launch_sim()
        
        self.port_self_ = port_self
        self.port_remote_ = port_remote

        self.fd_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fd_.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 112)       # set receive buffer size to be 112 bytes (1 double = 8 bytes)
        self.fd_.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 40)        # set send buffer size to be 40 bytes. Both 112 bytes and 40 bytes are lower than the minimum value, so buffer size will be set as minimum value. See "'man 7 socket"
        self.fd_.bind( ("", self.port_self_) )
        self.addr_remote_send_ = ("", self.port_remote_)
        buf = struct.pack('5d', 2, 0, 0, 0, 0)
        self.fd_.sendto(buf, self.addr_remote_send_)

        self.queue_command = Queue(3)
        self.path_all_ = np.zeros(shape=(14,8000,1))
        self.state_current = np.zeros(shape=(14))
        self.logDataFlag = False

        self.isRunning_ = True
        self.receiverThread = Thread(target=self.recvThread, args=())
        self.receiverThread.daemon = True
        self.receiverThread.start()
        self.senderThread = Thread(target=self.sendThread, args=())
        self.senderThread.daemon = True

        # #self.senderThread.start()
    
    def close_sim(self):
        os.killpg(self.controller_p_.pid, signal.SIGTERM)
        time.sleep(5)
        os.killpg(self.gazebo_p_.pid, signal.SIGTERM)
        time.sleep(5)  

    def delay_env_time(self,t_start,t_delay):
        # delay time defined in ms
        while (self.getTime() - t_start < t_delay/10000):
                    pass

    def __del__(self):
        self.isRunning_ = False
        self.fd_.close()


    def enableSticky(self, enable):      # enable=0 disable sticky, enable=1 enable sticky
        header = 11
        buf = struct.pack('5d', header, enable, 0, 0, 0)
        self.fd_.sendto(buf, self.addr_remote_send_)
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

        self.gazebo_p_ = subprocess.Popen(
            "gnome-terminal --disable-factory -e '/home/bhabas/catkin_ws/src/crazyflie_simulation/src/4.\ rl/src/launch_gazebo.bash'", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
        time.sleep(5)
        self.controller_p_ = subprocess.Popen(
            "gnome-terminal --disable-factory -e '/home/bhabas/catkin_ws/src/crazyflie_simulation/src/4.\ rl/src/launch_controller.bash'", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
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
        # time.sleep(0.1)
        return self.state_current
    
    def recvThread(self):
        print("Start recvThread")
        path = None
        
        k_run = 0
        while self.isRunning_:
            k_run = k_run + 1

            data, addr_remote_ = self.fd_.recvfrom(112)     # 1 double = 8 bytes
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
                self.path_all_ = np.append(self.path_all_, path, axis=2)
                path = None
                
            '''if k_run%50 == 1:
                # print("received data ", data, " from ", addr_remote_)
                print("=============================================================")
                print( 'Position: px=%.3f, py=%.3f, pz=%.3f' %(px, py, pz) )
                print( 'Orientat: q0=%.3f, q1=%.3f, q2=%.3f q3=%.3f' %(q0, q1, q2, q3) )
                print( 'Velocity: vx=%.3f, vy=%.3f, vz=%.3f' %(vx, vy, vz) )
                print( 'AnguRate: p =%.3f, q =%.3f, r =%.3f' %(p, q, r) )
                print( 'SimTime : time =%.3f' %(sim_time) )'''
            
            
    def sendThread(self):
        print("Start sendThread")
        while self.isRunning_:
            #time.sleep(0.5)
            #buf = struct.pack('5d', 2.0, 1.1, 2.2, 3.3, 4.4)
            buf = self.queue_command.get(block=True)
            len = self.fd_.sendto(buf, self.addr_remote_send_)
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
        self.fd_.sendto(buf, self.addr_remote_send_)
        #self.queue_command.put(buf, block=False)

        reward = 0
        done = 0
        info = 0
        return self.state_current, reward, done, info

    # def plotFigure(self):
    #     plt.figure()
    #     plt.plot(self.path_[0,:],self.path_[8,:], self.path_[0,:],self.path_[9,:], self.path_[0,:],self.path_[10,:])
    #     plt.show()
    
    def create_xls(self, start_time, sigma, alpha, file_name):
        file_log =  xlwt.Workbook()
        sheet = file_log.add_sheet('Learning process')
        sheet.write(0,0, start_time)
        sheet.write(0,1, 'sigma')
        sheet.write(0,2, sigma)
        sheet.write(0,3, 'alpha')
        sheet.write(0,4, alpha)

        sheet.write(1,0, 'Ep')
        sheet.write(1,1, 'Date')
        sheet.write(1,2, 'Time')
        sheet.write(1,3, 'Vz_d (m/s)')
        sheet.write(1,4, 'Vx_d (m/s)')
        sheet.write(1,5, 'RREV (rad/s)')
        sheet.write(1,6, 'omega_y (rad/s)')
        sheet.write(1,7, 'Action (q deg/s)')
        sheet.write(1,8, 'Reward')
        sheet.write(1,9, 'Distance (m)')
        sheet.write(1,10, 'Angle (deg)')
        sheet.write(1,11, 'Theta')
        file_log.save(file_name)

        return file_log, sheet

    
    def add_record_xls(self, file_log, sheet, file_name,
            k_ep, start_time1, start_time2,
            vz_ini, vx_ini, state, action, reward, info, theta):
        sheet.write(k_ep+2,0, k_ep+1)
        sheet.write(k_ep+2,1, start_time1)
        sheet.write(k_ep+2,2, start_time2)
        sheet.write(k_ep+2,3, np.around(vz_ini,3))
        sheet.write(k_ep+2,4, np.around(vx_ini,3))
        sheet.write(k_ep+2,5, np.around(state[0],3))
        sheet.write(k_ep+2,6, np.around(state[1],3))
        sheet.write(k_ep+2,7, np.around(action,3))
        sheet.write(k_ep+2,8, np.around(reward,3))
        sheet.write(k_ep+2,9, np.around(info[0],3))
        sheet.write(k_ep+2,10, np.around(info[1],3))
        sheet.write(k_ep+2,11, np.around(theta[0],3))
        sheet.write(k_ep+2,12, np.around(theta[1],3))
        sheet.write(k_ep+2,13, np.around(theta[2],3))
        file_log.save(file_name)
