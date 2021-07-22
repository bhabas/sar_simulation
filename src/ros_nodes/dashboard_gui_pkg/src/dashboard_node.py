import rospy
import numpy as np

from crazyflie_msgs.msg import RLData
from crazyflie_gazebo.msg import CtrlData
from nav_msgs.msg import Odometry

class DashboardNode:
    def __init__(self):
        print("[STARTING] Dashboard node is starting...")
        rospy.init_node("dashboard_gui_node")

        ## INITIALIZE CLASS VARIABLES
        # Publisher is very slow compared to Subscriber so this prevents calling uninitilized variables
        # while waiting to recieve them from the Publisher
        self.n_rollouts = 0
        self.k_run = 0
        self.k_ep = 0
        self.reward = 0
        self.state_current = np.zeros(14)
        
        

        ## INITIALIZE GLOBAL STATE SUBSCRIBER 
        rospy.Subscriber('/global_state',Odometry,self.global_stateCallback)

        ## INITIAILIZE REWARD SUBSCRIBER 
        rospy.Subscriber('/rl_data',RLData,self.rewardCallback)
        rospy.Subscriber('/ctrl_data',CtrlData,self.ctrlCallback)
        rospy.wait_for_message('/ctrl_data',CtrlData) # Wait to receive ctrl pub to run before continuing
   
        print("[COMPLETED] Dashboard node is running...")

    # ============================
    ##     Reward Subscriber
    # ============================
    def rewardCallback(self,reward_msg):
        
        ## SET CLASS VARIABLES TO MESSAGE VALUES
        self.k_run = reward_msg.k_run
        self.k_ep = reward_msg.k_ep
        self.reward = reward_msg.reward
        self.reward_avg = reward_msg.reward_avg
        self.n_rollouts = reward_msg.n_rollouts

    # ============================
    ##   Controller Subscriber
    # ============================

    def ctrlCallback(self,msg):
        self.MS = msg.motorspeeds
        self.FM = msg.FM
        self.FM_flip = msg.FM_flip
        

        



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


        ## COMBINE INTO COMPREHENSIVE LIST
        self.state_current = np.concatenate([np.atleast_1d(self.t),self.position,self.orientation_q,self.velocity,self.omega])
        