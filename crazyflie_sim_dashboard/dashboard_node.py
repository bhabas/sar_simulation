import rospy
import numpy as np

from crazyflie_msgs.msg import RLData
from crazyflie_msgs.msg import CtrlData
from nav_msgs.msg import Odometry

class DashboardNode:
    def __init__(self):
        print("[STARTING] Dashboard node is starting...")
        rospy.init_node("dashboard_gui_node",anonymous=True)

        ## INITIALIZE CLASS VARIABLES
        # Publisher is very slow compared to Subscriber so this prevents calling uninitilized variables
        # while waiting to recieve them from the Publisher
        self.n_rollouts = 0
        self.k_run = 0
        self.k_ep = 0


        self.k_ep_list1 = [0]
        self.k_ep_list2 = [0]
        self.k_ep_list3 = [0]


        self.r_list = [np.nan]
        self.r_avg_list = [np.nan]

        self.mu_list = np.array([[np.nan,np.nan]])
        self.sig_list = np.array([[np.nan,np.nan]])



        self.MS = [0,0,0,0]
        # a = np.append(a,[[5,6]],axis=0)
        

        ## INITIALIZE GLOBAL STATE SUBSCRIBER 
        rospy.Subscriber('/env/vicon_state',Odometry,self.global_stateCallback)

        ## INITIAILIZE REWARD SUBSCRIBER 
        rospy.Subscriber('/rl_data',RLData,self.rewardCallback)
        rospy.Subscriber('/ctrl_data',CtrlData,self.ctrlCallback)
        rospy.wait_for_message('/ctrl_data',CtrlData) # Wait to receive ctrl pub to run before continuing
   
        print("[COMPLETED] Dashboard node is running...")

    # ============================
    ##     Reward Subscriber
    # ============================
    def rewardCallback(self,reward_msg):

        self.n_rollouts = reward_msg.n_rollouts

        ## ON FIRST ROLLOUTS
        if self.k_run != reward_msg.k_run and reward_msg.k_run == 0:

        # a = np.append(a,[[5,6]],axis=0)
            self.mu_list = np.append(self.mu_list,[[reward_msg.mu[0],reward_msg.mu[1]]],axis=0)
            self.sig_list = np.append(self.sig_list,[[reward_msg.sigma[0],reward_msg.sigma[1]]],axis=0)


            self.k_ep_list3.append(reward_msg.k_ep)

        
        ## UPDATE EPISODE AND RUN NUMBER
        if self.k_run != reward_msg.k_run:

            self.k_run = reward_msg.k_run
            self.k_ep = reward_msg.k_ep
            
        ## IF REWARD CHANGES THEN APPEND NEW REWARD TO LIST
        if self.r_list[-1] != reward_msg.reward:
            self.k_ep_list1.append(reward_msg.k_ep)
            self.r_list.append(reward_msg.reward)

        ## IF FINAL RUN, THEN APPEND REWARD AVG TO LIST
        if self.k_run == self.n_rollouts-1:
            self.r_avg_list.append(reward_msg.reward_avg)
            self.k_ep_list2.append(self.k_ep)



            


     

        

    # ============================
    ##   Controller Subscriber
    # ============================

    def ctrlCallback(self,msg):
        self.FM = msg.FM
        self.MS_PWM = msg.MS_PWM

        self.RREV = np.nan
        # self.tau = 1/msg.RREV
        self.OF_y = msg.OF_y
        self.OF_x = msg.OF_x    
        

        



    # ============================
    ##   Global State Subscriber
    # ============================
    def global_stateCallback(self,gs_msg):

        ## SET TIME VALUE FROM TOPIC
        t_temp = gs_msg.header.stamp.secs
        ns_temp = gs_msg.header.stamp.nsecs
        self.t = np.round(t_temp+ns_temp*1e-9,3) # (seconds + nanoseconds)
        
        ## SIMPLIFY STATE VALUES FROM TOPIC
        global_pos = gs_msg.pose.pose.position
        global_quat = gs_msg.pose.pose.orientation
        global_vel = gs_msg.twist.twist.linear
        global_omega = gs_msg.twist.twist.angular
        
        if global_quat.w == 0: # If zero at startup set quat.w to one to prevent errors
            global_quat.w = 1

        ## SET STATE VALUES FROM TOPIC
        
        self.position = np.round([global_pos.x,global_pos.y,global_pos.z],3)
        self.orientation_q = np.round([global_quat.w,global_quat.x,global_quat.y,global_quat.z],3)
        self.velocity = np.round([global_vel.x,global_vel.y,global_vel.z],3)
        self.omega = np.round([global_omega.x,global_omega.y,global_omega.z],3)

if __name__ == "__main__":
    dash = DashboardNode()
    rospy.spin()