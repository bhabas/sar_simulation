import rospy
import message_filters
from threading import Thread

import numpy as np

from gazebo_communication_pkg.msg import GlobalState


import csv

# from ros_nodes.datalogging_pkg.src.dataLogger import DLNode

class DataLoggingNode:
    def __init__(self):
        print("[STARTING] DataLogging node is starting...")
        rospy.init_node("dataLogging_node")

        self.t_step = 0
        self.create_flag = True
        self.logging_flag = True

        self.StateSub = message_filters.Subscriber("/global_state",GlobalState)
        self.atsSub()



    def atsSub(self):
        ats = message_filters.ApproximateTimeSynchronizer([self.StateSub],queue_size=5,slop=0.1)
        ats.registerCallback(self.csvWriter)
        rospy.spin()

    def csvWriter(self,gs_msg):

        self.t_step += 1
        
        ## SET TIME VALUE FROM GLOBAL_STATE TOPIC
        t_temp = gs_msg.header.stamp.secs
        ns_temp = gs_msg.header.stamp.nsecs
        self.t = t_temp+ns_temp*1e-9 # (seconds + nanoseconds)
        
        ## SIMPLIFY STATE VALUES FROM GLOBAL_STATE TOPIC
        global_pos = gs_msg.global_pose.position
        global_quat = gs_msg.global_pose.orientation
        global_vel = gs_msg.global_twist.linear
        global_omega = gs_msg.global_twist.angular

        ## SET STATE VALUES FROM TOPIC
        self.position = np.asarray([global_pos.x,global_pos.y,global_pos.z])
        self.orientation_q = np.asarray([global_quat.w,global_quat.x,global_quat.y,global_quat.z])
        self.velocity = np.asarray([global_vel.x,global_vel.y,global_vel.z])
        self.omega = np.asarray([global_omega.x,global_omega.y,global_omega.z])

        ## TRIM STATE VALUES FOR CSV
        self.position = np.round(self.position,3)
        self.orientation_q = np.round(self.orientation_q,3)
        self.velocity = np.round(self.velocity,3)
        self.omega = np.round(self.omega,3)



        if self.logging_flag == True:

            # if self.trial_name != rl_msg.trial_name:
            if self.create_flag == True:
                self.create_csv()
                self.create_flag = False

            if self.t_step%5 == 0: # Slow down recording by x5
                self.append_csv()



            # if rollout == complete:
            #     self.append_IC()




    def create_csv(self):
        self.path =  "/home/bhabas/catkin_ws/src/crazyflie_simulation/src/ros_nodes/data_logging_pkg/log/test.csv"

        with open(self.path,mode='w') as state_file:
            state_writer = csv.writer(state_file,delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
            state_writer.writerow([
                'k_ep','k_run',
                'alpha_mu','alpha_sig',
                'mu','sigma', 'policy',
                't','x','y','z',
                'qx','qy','qz','qw',
                'vx','vy','vz',
                'wx','wy','wz',
                'gamma','reward','flip_trigger','n_rollouts',
                'RREV','OF_x','OF_y',
                "","","","", ])


    def append_csv(self):

        with open(self.path, mode='a') as state_file:
            state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            state_writer.writerow([
                "","",
                "","", # alpha_mu,alpha_sig
                "","","", # mu,sigma,policy
                self.t,self.position[0],self.position[1],self.position[2], # t,x,y,z
                self.orientation_q[0],self.orientation_q[1],self.orientation_q[2],self.orientation_q[3], # qw,qx,qy,qz
                self.velocity[0],self.velocity[1],self.velocity[2], # vx,vy,vz
                self.omega[0],self.omega[1],self.omega[2], # wx,wy,wz
                "","","flip_triggered","", # gamma, reward, flip_triggered, n_rollout
                "RREV","OF_x","OF_y", # RREV, OF_x, OF_y
                "","","","", # Place holders
                ]) 
        

    # def append_IC(self):

    #     with open(self.path, mode='a') as state_file:
    #         state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #         state_writer.writerow([
    #             self.k_ep,self.k_run,
    #             self.alpha_mu,self.alpha_sigma, # alpha_mu,alpha_sig
    #             self.mu,self.sigma,self.policy, # mu,sigma,policy
    #             "","","","", # t,x,y,z
    #             "", "", "", "", # qx,qy,qz,qw
    #             "vdx","vdy","vdz", # vx,vy,vz
    #             "wdx","wdy","wdz", # wx,wy,wz
    #             self.gamma,self.reward,"",self.n_rollouts, # gamma, reward, flip_triggered, n_rollout
    #             "","","", # RREV, OF_x, OF_y
    #             "","","","", # Place holders
    #             ])
        
    def append_csv_blank(self): 
        
         with open(self.path, mode='a') as state_file:
            state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            state_writer.writerow([])
        

        



        
if __name__ == "__main__":
    DLNode = DataLoggingNode()
    # print("struf")
    while not rospy.is_shutdown():
        pass



