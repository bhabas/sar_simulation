#!/usr/bin/env python3
import rospy,message_filters
import csv,getpass
import numpy as np



from gazebo_communication_pkg.msg import GlobalState
from crazyflie_rl.msg import RLData
from crazyflie_gazebo.msg import CtrlData




class DataLoggingNode:
    def __init__(self):
        print("[STARTING] DataLogging node is starting...")
        rospy.init_node("dataLogging_node")

        self.t_step = 0
        self.k_run_temp = 0
        self.createCSV_flag = True
        self.logging_flag = True

        self.StateSub = message_filters.Subscriber("/global_state",GlobalState)
        self.RLSub = message_filters.Subscriber("/rl_data",RLData)
        self.CtrlSub = message_filters.Subscriber("/ctrl_data",CtrlData)
        self.atsSub()


    def atsSub(self):
        ats = message_filters.ApproximateTimeSynchronizer([self.RLSub,self.StateSub,self.CtrlSub],queue_size=5,slop=0.05,allow_headerless=True)
        ats.registerCallback(self.csvWriter)
        rospy.spin()

    def csvWriter(self,rl_msg,gs_msg,ctrl_msg):

        self.t_step += 1

        ## SET TIME VALUE FROM GLOBAL_STATE TOPIC
        t_temp = gs_msg.header.stamp.secs
        ns_temp = gs_msg.header.stamp.nsecs
        self.t = t_temp+ns_temp*1e-9 # (seconds + nanoseconds)
        self.t = np.round(self.t,3)

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

         ## TEMP SENSOR VALUES
        self.RREV = np.round(gs_msg.RREV,2)
        self.OF_x = np.round(gs_msg.OF_x,2)
        self.OF_y = np.round(gs_msg.OF_y,2)



        ## SET RL PARAMS FROM RL_DATA TOPIC
        self.trial_name = rl_msg.trial_name
        self.agent = rl_msg.agent

        self.logging_flag = rl_msg.logging_flag
        self.createCSV_flag = rl_msg.createCSV_flag
        self.runComplete_flag = rl_msg.runComplete_flag

        self.n_rollouts = rl_msg.n_rollouts
        self.gamma = np.round(rl_msg.gamma,2)
        self.h_ceiling = rl_msg.h_ceiling

        self.k_ep = rl_msg.k_ep
        self.k_run = rl_msg.k_run

        ## SET RL VALUES FROM TOPIC
        self.alpha_mu = np.asarray(rl_msg.alpha_mu)
        self.alpha_sigma = np.asarray(rl_msg.alpha_sigma)
        self.mu = np.asarray(rl_msg.mu)
        self.sigma = np.asarray(rl_msg.sigma)
        self.policy = np.asarray(rl_msg.policy)
        self.vel_d = np.asarray(rl_msg.vel_d)
        self.omega_d = np.asarray(rl_msg.M_d)

        ## TRIM RL VALUES FOR CSV
        self.alpha_mu = np.round(self.alpha_mu,2)
        self.alpha_sigma = np.round(self.alpha_sigma,2)
        self.mu = np.round(self.mu,2)
        self.sigma = np.round(self.sigma,2)
        self.policy = np.round(self.policy,2)
        self.vel_d = np.round(self.vel_d,2)
        self.omega_d = np.round(self.omega_d,2)

        self.reward = np.round(rl_msg.reward,3)


       
        
        ## SET & TRIM CTRL VALUES FROM CTRL_DATA TOPIC
        self.MS = np.asarray(ctrl_msg.motorspeeds)
        self.MS = np.round(self.MS,0)

        self.FM = np.asarray(ctrl_msg.FM)
        self.FM = np.round(self.FM,2)

        self.FM_flip = np.asarray(ctrl_msg.FM_flip)
        self.FM_flip = np.round(self.FM_flip,2)

        


        self.flip_flag = ctrl_msg.flip_flag
        self.RREV_tr = np.round(ctrl_msg.RREV_tr,2)
        self.OF_y_tr = np.round(ctrl_msg.OF_y_tr,2)




        username = getpass.getuser()
        self.path =  f"/home/{username}/catkin_ws/src/crazyflie_simulation/src/ros_nodes/data_logging_pkg/log/{self.trial_name}.csv"



        if self.logging_flag == True:

            if self.createCSV_flag == True:
                self.create_csv()

            if self.k_run_temp != rl_msg.k_run: # When k_run changes then add blank row
                self.append_csv_blank()
                self.k_run_temp = rl_msg.k_run

            if self.t_step%1 == 0: # Slow down recording by [x5]
                self.append_csv()

            if self.runComplete_flag == True:
                self.append_IC()







    def create_csv(self):

        with open(self.path,mode='w') as state_file:
            state_writer = csv.writer(state_file,delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
            state_writer.writerow([
                'k_ep','k_run',
                'alpha_mu','alpha_sig',
                'mu','sigma', 'policy',
                't','x','y','z',
                'qw','qx','qy','qz',
                'vx','vy','vz',
                'wx','wy','wz',
                'gamma','reward','flip_trigger','n_rollouts',
                'RREV','OF_x','OF_y',
                'MS1','MS2','MS3','MS4',
                'F_thrust','Mx','My','Mz'])# Place holders


    def append_csv(self):

        with open(self.path, mode='a') as state_file:
            state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            state_writer.writerow([
                self.k_ep,self.k_run,
                "","", # alpha_mu,alpha_sig
                "","","", # mu,sigma,policy
                self.t,self.position[0],self.position[1],self.position[2], # t,x,y,z
                self.orientation_q[0],self.orientation_q[1],self.orientation_q[2],self.orientation_q[3], # qw,qx,qy,qz
                self.velocity[0],self.velocity[1],self.velocity[2], # vx,vy,vz
                self.omega[0],self.omega[1],self.omega[2], # wx,wy,wz
                "","",self.flip_flag,"", # gamma, reward, flip_triggered, n_rollout
                self.RREV,self.OF_x,self.OF_y, # RREV, OF_x, OF_y
                self.MS[0],self.MS[1],self.MS[2],self.MS[3],
                self.FM[0],self.FM[1],self.FM[2],self.FM[3] # Place holders
                ])


    def append_IC(self):

        with open(self.path, mode='a') as state_file:
            state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            state_writer.writerow([
                self.k_ep,self.k_run,
                self.alpha_mu,self.alpha_sigma, # alpha_mu,alpha_sig
                self.mu,self.sigma,self.policy, # mu,sigma,policy
                "","","","", # t,x,y,z
                "", "", "", "", # qx,qy,qz,qw
                self.vel_d[0],self.vel_d[1],self.vel_d[2], # vx,vy,vz
                self.omega_d[0],self.omega_d[1],self.omega_d[2], # wx,wy,wz
                self.gamma,self.reward,"",self.n_rollouts, # gamma, reward, flip_triggered, n_rollout
                self.RREV_tr,"",self.OF_y_tr, # RREV, OF_x, OF_y
                "","","","",
                "",self.FM_flip[1],self.FM_flip[2],self.FM_flip[3], # Place holders Include successful run flag
                ])

    def append_csv_blank(self):

         with open(self.path, mode='a') as state_file:
            state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            state_writer.writerow([])







if __name__ == "__main__":
    DLNode = DataLoggingNode()
    while not rospy.is_shutdown():
        pass



