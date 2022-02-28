import rospy
import numpy as np

from crazyflie_msgs.msg import RLData,RLConvg
from crazyflie_msgs.msg import CtrlData
from sensor_msgs.msg import LaserScan
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


        self.r_list = [np.nan]
        self.r_avg_list = [np.nan]


        self.mu_1_list = [] # I can generalize this if needed
        self.mu_2_list = []
        self.sigma_1_list = []
        self.sigma_2_list = []

        self.pos = np.array([0,0,0])
        self.vel = np.array([0,0,0])
        self.quat = np.array([0,0,0,1])
        self.eul = np.array([0,0,0])
        self.omega = np.array([0,0,0])
        self.MS = np.array([0,0,0,0])
        

        ## INITIALIZE GLOBAL STATE SUBSCRIBER 
        rospy.Subscriber('/env/vicon_state',Odometry,self.global_stateCallback)

        ## INITIAILIZE REWARD SUBSCRIBER 
        rospy.Subscriber('/rl_data',RLData,self.rewardCallback,queue_size=10)
        rospy.Subscriber('/ctrl_data',CtrlData,self.ctrlCallback,queue_size=1)
        rospy.Subscriber('/cf1/laser',LaserScan,self.laserCallback,queue_size=1)
        rospy.Subscriber('/rl_convg',RLConvg,self.rlConvgCallback,queue_size=10)
        rospy.wait_for_message('/ctrl_data',CtrlData) # Wait to receive ctrl pub to run before continuing
   
        print("[COMPLETED] Dashboard node is running...")

    def printSumthin(self):
        print("Hello")

    # ============================
    ##     Reward Subscriber
    # ============================
    def rewardCallback(self,reward_msg):

        self.n_rollouts = reward_msg.n_rollouts        
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


    def rlConvgCallback(self,msg):
        self.mu_1_list = np.array(msg.mu_1_list)
        self.mu_2_list = np.array(msg.mu_2_list)
        self.sigma_1_list = np.array(msg.sigma_1_list)
        self.sigma_2_list = np.array(msg.sigma_2_list)


    
    def laserCallback(self,msg):
        self.d_ceiling = msg.ranges[0]
        

     

        

    # ============================
    ##   Controller Subscriber
    # ============================

    def ctrlCallback(self,msg):
        self.FM = msg.FM
        self.MS_PWM = msg.MS_PWM

        self.RREV = np.nan
        # self.tau = 1/msg.RREV
        self.OFy = msg.OFy
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
        self.pos = np.round([global_pos.x,global_pos.y,global_pos.z],3)
        self.vel = np.round([global_vel.x,global_vel.y,global_vel.z],3)
        self.quat = np.round([global_quat.x,global_quat.y,global_quat.z,global_quat.w],3)
        self.eul = np.round(self.quat2euler(self.quat,degrees=True),3)
        self.omega = np.round([global_omega.x,global_omega.y,global_omega.z],3)
        
    def quat2euler(self,quat,degrees=True):
        """Converts quaternion into euler angles in [YZX] notation

        Args:
            quat (np.array): Quaternion in [x,y,z,w] format

        Returns:
            phi,theta,psi: Euler angles
        """        

        ## CALC NEEDED ROTATION MATRIX COMPONENTS FROM QUATERNION
        R11 = 1.0 - 2.0*( pow(quat[1],2) + pow(quat[2],2) )
        R21 = 2.0*(quat[0]*quat[1] + quat[2]*quat[3])
        R31 = 2.0*(quat[0]*quat[2] - quat[1]*quat[3])

        R22 = 1.0 - 2.0*( pow(quat[0],2) + pow(quat[2],2) )
        R23 = 2.0*(quat[1]*quat[2] - quat[0]*quat[3])

        ## CONVERT ROTATION MATRIX COMPONENTS TO EULER ANGLES (YZX NOTATION)
        phi = np.arctan2(-R23, R22) # X-axis
        theta = np.arctan2(-R31, R11) # Y-axis
        psi = np.arcsin(R21) # Z-axis

        if degrees == True:
            return np.degrees(phi),np.degrees(theta),np.degrees(psi)
        else:
            return phi,theta,psi



if __name__ == "__main__":
    dash = DashboardNode()
    rospy.spin()