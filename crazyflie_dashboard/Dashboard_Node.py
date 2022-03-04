import rospy
import numpy as np

from crazyflie_msgs.msg import CF_StateData,CF_MiscData,CF_FlipData,CF_ImpactData
from crazyflie_msgs.msg import RLData,RLConvg


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

        ## INITIALIZE STATE VALUES
        self.t = 0.0
        self.posCF = [0,0,0]
        self.velCF = [0,0,0]

        self.quatCF = [0,0,0,1]
        self.eulCF = [0,0,0]
        self.omegaCF = [0,0,0]
        self.eulCF = [0,0,0]

        self.Tau = 0.0
        self.OFx = 0.0
        self.OFy = 0.0
        self.RREV = 0.0
        self.d_ceil = 0.0 

        self.MS_pwm = [0,0,0,0] # Controller Motor Speeds (MS1,MS2,MS3,MS4) [PWM]
        self.FM = [0,0,0,0]     # Controller Force/Moments (F_thrust,Mx,My,Mz) [N,N*mm]
        
        self.NN_flip = 0.0
        self.NN_policy = 0.0
        
        self.x_d = [0,0,0]
        self.v_d = [0,0,0]
        self.a_d = [0,0,0]

        ## INITIALIZE FLIP VALUES
        self.flip_flag = False      # Flag if model has started flip maneuver

        self.t_tr = 0.0             # [s]
        self.posCF_tr = [0,0,0]     # [m]
        self.velCF_tr = [0,0,0]     # [m/s]
        self.quatCF_tr = [0,0,0,1]  # [quat]
        self.omegaCF_tr = [0,0,0]   # [rad/s]
        self.eulCF_tr = [0,0,0]

        self.Tau_tr = 0.0
        self.RREV_tr = 0.0          # [rad/s]
        self.OFx_tr = 0.0           # [rad/s]
        self.OFy_tr = 0.0           # [rad/s]
        self.d_ceil_tr = 0.0        # [m]

        self.FM_tr = [0,0,0,0]      # [N,N*mm]

        self.NN_tr_flip = 0.0
        self.NN_tr_policy = 0.0     # [N*mm]

        ## INITIALIZE IMPACT VALUES
        self.impact_flag = False
        self.BodyContact_flag = False   # Flag if model body impacts ceiling plane

        self.t_impact = 0.0
        self.posCF_impact = [0,0,0]
        self.velCF_impact = [0,0,0]
        self.quatCF_impact = [0,0,0,1]
        self.omegaCF_impact = [0,0,0]
        self.eulCF_impact = [0,0,0]

        self.impact_force_x = 0.0     # Ceiling impact force, X-dir [N]
        self.impact_force_y = 0.0     # Ceiling impact force, Y-dir [N]
        self.impact_force_z = 0.0     # Ceiling impact force, Z-dir [N]
        self.impact_magnitude = 0.0

        self.pad_connections = 0 # Number of pad connections

        ## INITIALIZE MISC VALUES
        self.V_Battery = 0.0




        ## INITIALIZE GLOBAL STATE SUBSCRIBER 
        # rospy.Subscriber("/UKF/viconState_Filtered",Vicon_Filtered,self.viconFilteredCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/MiscData",CF_MiscData,self.CF_MiscDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/FlipData",CF_FlipData,self.CF_FlipDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/ImpactData",CF_ImpactData,self.CF_ImpactDataCallback,queue_size=1)


        ## INITIAILIZE REWARD SUBSCRIBER 
        rospy.Subscriber('/RL/rl_data',RLData,self.rewardCallback,queue_size=10)
        rospy.Subscriber('/RL/rl_convg',RLConvg,self.rlConvgCallback,queue_size=10)
   
        print("[COMPLETED] Dashboard node is running...")


    def viconFilteredCallback(self,vicon_msg):
        self.posVicon = np.round([vicon_msg.pos.x,vicon_msg.pos.y,vicon_msg.pos.z],3)
        self.velVicon = np.round([np.nan,np.nan,np.nan],3)
        self.eulVicon = np.round([vicon_msg.eul.x,vicon_msg.eul.y,vicon_msg.eul.z],3)
        # self.eulVicon = np.round([np.nan,np.nan,vicon_msg.eul.z],3)


    def CF_StateDataCallback(self,StateData_msg):

        self.t = np.round(StateData_msg.header.stamp.to_sec(),4)

        self.posCF = np.round([ StateData_msg.Pose.position.x,
                                StateData_msg.Pose.position.y,
                                StateData_msg.Pose.position.z],3)

        

        self.quatCF = np.round([StateData_msg.Pose.orientation.x,
                                StateData_msg.Pose.orientation.y,
                                StateData_msg.Pose.orientation.z,
                                StateData_msg.Pose.orientation.w],3)

        self.eulCF = np.round([ StateData_msg.Eul.x,
                                StateData_msg.Eul.y,
                                StateData_msg.Eul.z],3)

        ## CF_TWIST
        self.velCF = np.round([ StateData_msg.Twist.linear.x,
                                StateData_msg.Twist.linear.y,
                                StateData_msg.Twist.linear.z],3)

        self.omegaCF = np.round([StateData_msg.Twist.angular.x,
                                 StateData_msg.Twist.angular.y,
                                 StateData_msg.Twist.angular.z],3)

        ## CF_VISUAL STATES
        self.Tau = np.round(StateData_msg.Tau,3)
        self.OFx = np.round(StateData_msg.OFx,3)
        self.OFy = np.round(StateData_msg.OFy,3)
        self.RREV = np.round(StateData_msg.RREV,3)
        self.d_ceil = np.round(StateData_msg.D_ceil,3)

        ## CONTROLLER ACTIONS
        self.FM = np.round([StateData_msg.FM[0],
                            StateData_msg.FM[1],
                            StateData_msg.FM[2],
                            StateData_msg.FM[3]],3)

        self.MS_PWM = np.round([StateData_msg.MS_PWM[0],
                                StateData_msg.MS_PWM[1],
                                StateData_msg.MS_PWM[2],
                                StateData_msg.MS_PWM[3]],0)

        self.NN_flip = np.round(StateData_msg.NN_flip,3)
        self.NN_policy = np.round(StateData_msg.NN_policy,3)

        self.x_d = np.round([StateData_msg.x_d.x,
                             StateData_msg.x_d.y,
                             StateData_msg.x_d.z],3)

        self.v_d = np.round([StateData_msg.v_d.x,
                             StateData_msg.v_d.y,
                             StateData_msg.v_d.z],3)

        self.a_d = np.round([StateData_msg.a_d.x,
                             StateData_msg.a_d.y,
                             StateData_msg.a_d.z],3)


    def CF_FlipDataCallback(self,FlipData_msg):
        ## FLIP FLAGS
        self.flip_flag = FlipData_msg.flip_flag

        self.t_tr = np.round(FlipData_msg.header.stamp.to_sec(),4)

        ## CF_POSE (FLIP)
        self.posCF_tr = np.round([FlipData_msg.Pose_tr.position.x,
                                  FlipData_msg.Pose_tr.position.y,
                                  FlipData_msg.Pose_tr.position.z],3)

        self.quatCF_tr = np.round([FlipData_msg.Pose_tr.orientation.x,
                                   FlipData_msg.Pose_tr.orientation.y,
                                   FlipData_msg.Pose_tr.orientation.z,
                                   FlipData_msg.Pose_tr.orientation.w],3)

        self.eulCF_tr = np.round([FlipData_msg.Eul_tr.x,
                                  FlipData_msg.Eul_tr.y,
                                  FlipData_msg.Eul_tr.z],3)

        ## CF_TWIST (FLIP)
        self.velCF_tr = np.round([FlipData_msg.Twist_tr.linear.x,
                                  FlipData_msg.Twist_tr.linear.y,
                                  FlipData_msg.Twist_tr.linear.z],3)

        self.omegaCF_tr = np.round([FlipData_msg.Twist_tr.angular.x,
                                    FlipData_msg.Twist_tr.angular.y,
                                    FlipData_msg.Twist_tr.angular.z],3)

        ## CF_VISUAL STATES (FLIP)
        self.Tau_tr = np.round(FlipData_msg.Tau_tr,3)
        self.OFx_tr = np.round(FlipData_msg.OFx_tr,3)
        self.OFy_tr = np.round(FlipData_msg.OFy_tr,3)
        self.RREV_tr = np.round(FlipData_msg.RREV_tr,3)
        self.d_ceil_tr = np.round(FlipData_msg.D_ceil_tr,3)

        ## CONTROLLER ACTIONS
        self.FM_tr = np.round([FlipData_msg.FM_tr[0],
                               FlipData_msg.FM_tr[1],
                               FlipData_msg.FM_tr[2],
                               FlipData_msg.FM_tr[3]],3)


        self.NN_tr_flip = np.round(FlipData_msg.NN_tr_flip,3)
        self.NN_tr_policy = np.round(FlipData_msg.NN_tr_policy,3)

    def CF_ImpactDataCallback(self,ImpactData_msg):
        ## IMPACT FLAGS
        self.impact_flag = ImpactData_msg.impact_flag
        self.BodyContact_flag = ImpactData_msg.BodyContact_flag

        self.t_impact = np.round(ImpactData_msg.header.stamp.to_sec(),4)

        ## CF_POSE (IMPACT)
        self.posCF_impact = np.round([ImpactData_msg.Pose_impact.position.x,
                                      ImpactData_msg.Pose_impact.position.y,
                                      ImpactData_msg.Pose_impact.position.z],3)

        

        self.quatCF_impact = np.round([ImpactData_msg.Pose_impact.orientation.x,
                                       ImpactData_msg.Pose_impact.orientation.y,
                                       ImpactData_msg.Pose_impact.orientation.z,
                                       ImpactData_msg.Pose_impact.orientation.w],3)

        self.eulCF_impact = np.round([ImpactData_msg.Eul_impact.x,
                                      ImpactData_msg.Eul_impact.y,
                                      ImpactData_msg.Eul_impact.z],3)

        ## CF_TWIST (IMPACT)
        self.velCF_impact = np.round([ImpactData_msg.Twist_impact.linear.x,
                                      ImpactData_msg.Twist_impact.linear.y,
                                      ImpactData_msg.Twist_impact.linear.z],3)

        self.omegaCF_impact = np.round([ImpactData_msg.Twist_impact.angular.x,
                                        ImpactData_msg.Twist_impact.angular.y,
                                        ImpactData_msg.Twist_impact.angular.z],3)

        ## IMPACT FORCES
        self.Force_impact = np.round([ImpactData_msg.Force_impact.x,
                                      ImpactData_msg.Force_impact.y,
                                      ImpactData_msg.Force_impact.z],3)

        self.impact_magnitude = np.round(ImpactData_msg.Impact_Magnitude,3)

        ## STICKY PAD CONNECTIONS
        self.pad_connections = ImpactData_msg.Pad_Connections
        self.Pad1_Contact = ImpactData_msg.Pad1_Contact
        self.Pad2_Contact = ImpactData_msg.Pad2_Contact
        self.Pad3_Contact = ImpactData_msg.Pad3_Contact
        self.Pad4_Contact = ImpactData_msg.Pad4_Contact


    def CF_MiscDataCallback(self,MiscData_msg):
        
        self.V_battery = np.round(MiscData_msg.battery_voltage,4)

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



     



if __name__ == "__main__":
    dash = DashboardNode()
    rospy.spin()