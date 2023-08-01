#!/usr/bin/env python3
import numpy as np
import os
import rospy
import getpass
import time

## ROS MESSAGES AND SERVICES
from sar_msgs.msg import SAR_StateData,SAR_TriggerData,SAR_ImpactData,SAR_MiscData
from sar_msgs.srv import Logging_CMD,Logging_CMDRequest
from sar_msgs.srv import CTRL_Cmd_srv,CTRL_Cmd_srvRequest
from sar_msgs.msg import RL_Data,RL_History

from rosgraph_msgs.msg import Clock

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
ENDC = '\033[m'

class SAR_Base_Interface():

    def __init__(self,Exp_Flag=False):
        os.system("roslaunch sar_launch Load_Params.launch")      
        if not Exp_Flag:   
            rospy.init_node("SAR_Env_Node")


        ## SAR PARAMETERS
        self.SAR_Type = rospy.get_param('/SAR_SETTINGS/SAR_Type')
        self.SAR_Config = rospy.get_param('/SAR_SETTINGS/SAR_Config')
        self.modelInitials = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Initials")
        self.modelName = f"{self.SAR_Type}_{self.SAR_Config}"
        self.preInit_Values()

        self.pos_0 = [0.0, 0.0, 0.4]      # Default hover position [m]
        self.accCF_max = [1.0, 1.0, 3.1]    # Max acceleration values for trajectory generation [m/s^2]


        ## PLANE PARAMETERS
        self.Plane_Model = rospy.get_param('/PLANE_SETTINGS/Plane_Model')
        
               

        ## INIT LOGGING VALUES
        self.username = getpass.getuser()
        self.logDir =  f"/home/{self.username}/catkin_ws/src/sar_simulation/sar_logging/local_logs"
        self.logName = "TestLog.csv"
        self.error_str = "No_Debug_Data"



        ## CRAZYFLIE DATA SUBSCRIBERS 
        # NOTE: Queue sizes=1 so that we are always looking at the most current data and 
        #       not data at back of a queue waiting to be processed by callbacks
        rospy.Subscriber("/clock",Clock,self.clockCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/StateData",SAR_StateData,self.SAR_StateDataCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/TriggerData",SAR_TriggerData,self.SAR_TriggerDataCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/ImpactData",SAR_ImpactData,self.SAR_ImpactDataCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/MiscData",SAR_MiscData,self.SAR_MiscDataCallback,queue_size=1)

        ## RL DATA PUBLISHERS
        self.RL_Data_Publisher = rospy.Publisher('/RL/Data',RL_Data,queue_size=10)
        self.RL_Convg_Publisher = rospy.Publisher('/RL/History',RL_History,queue_size=10)


    def getTime(self):
        """Returns current known time.

        Returns:
            float: Current known time.
        """        
        
        return self.t

    def SendCmd(self,action,cmd_vals=[0,0,0],cmd_flag=1):
        """Sends commands to SAR_DC->Controller via rosservice call

        Args:
            action (string): The desired command
            cmd_vals (list, optional): Command values typically in [x,y,z] notation. Defaults to [0,0,0].
            cmd_flag (float, optional): Used as either a on/off flag for command or an extra float value if needed. Defaults to 1.
        """        

        cmd_dict = {
            'Ctrl_Reset':0,
            'Pos':1,
            'Vel':2,
            'Yaw':3,
            'Stop':5,
            'Moment':7,
            'Policy':8,

            'P2P_traj':10,
            'Vel_traj':11,
            'Impact_traj':12,

            'Tumble':20,
            'Load_Params':21,
            'Cap_Logging':22,

            'Thrust':30,
            'PWM':31,

            'GZ_Pose_Reset':90,
            'GZ_Const_Vel_Traj':91,
            'GZ_StickyPads':92,
            'GZ_Plane_Pose':93,
        }

        ## CREATE SERVICE REQUEST MSG
        srv = CTRL_Cmd_srvRequest() 
        
        srv.cmd_type = cmd_dict[action]
        srv.cmd_vals.x = cmd_vals[0]
        srv.cmd_vals.y = cmd_vals[1]
        srv.cmd_vals.z = cmd_vals[2]
        srv.cmd_flag = cmd_flag
        srv.cmd_rx = True

        self.callService('/SAR_DC/CMD_Input',srv,CTRL_Cmd_srv)    

    
    def callService(self,srv_addr,srv_msg,srv_type,num_retries=5):

        ## CHECK THAT SERVICE IS AVAILABLE
        try:
            rospy.wait_for_service(srv_addr, 1)

        except rospy.ROSException as e:
            rospy.logerr(f"[WARNING] Service '{srv_addr}' not available: {e}")
            return None
            
        ## CALL SERVICE AND RETURN RESPONSE
        service_proxy = rospy.ServiceProxy(srv_addr, srv_type)
        for retry in range(num_retries):
            try:
                response = service_proxy(srv_msg)
                return response
            
            except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
                rospy.logwarn(f"[WARNING] Attempt {retry + 1} to call service '{srv_addr}' failed: {e}")

        ## IF SERVICE CALL FAILS THEN MARK SIM EPISODE AS DONE AND RETURN ERROR
        self.Done = True
        rospy.logerr(f"Service '{srv_addr}' call failed after {num_retries} attempts")
        return None

                

    def VelTraj_StartPos(self,x_impact,V_d,accel_d=None,Tau_0=0.5):
        """Returns the required start position (x_0,z_0) to intercept the 180 deg ceiling 
        at a specific x-location; while also achieving the desired velocity conditions 
        at by a certain distance from the ceiling.

        Args:
            x_impact (float): X-location to impact the ceiling
            V_d (list): List of desired velocity components [Vx,Vy,Vz]
            accel_d (list, optional): List of acceleration components [ax,ay,az]. Defaults to env values if (None).
            d_vz (float, optional): Distance from ceiling at which velocity conditions are met. Defaults to 0.6 m.

        Returns:
            x_0: Start x-location for trajectory
            z_0: Start z-location for trajectory
        """        

        ## DESIRED VELOCITY VALUES
        Vx = V_d[0]
        Vz = V_d[2]

        ## DEFAULT TO TAU BASED MIN DISTANCE
        d_vz = Tau_0*Vz

        ## DEFAULT TO CLASS VALUES
        if accel_d == None:
            accel_d = self.accCF_max
    
        a_x = accel_d[0]
        a_z = accel_d[2]


        ## CALC OFFSET POSITIONS
        t_x = Vx/a_x    # Time required to reach Vx
        t_z = Vz/a_z    # Time required to reach Vz

        z_vz = 0.5*a_z*(t_z)**2                 # Height Vz reached
        z_0 = (2.10 - d_vz) - z_vz    # Offset to move z_vz to d_vz
        
        x_vz = Vx*(t_x+t_z) - Vx**2/(2*a_x)     # X-position Vz reached
        x_0 = x_impact - x_vz - d_vz*Vx/Vz      # Account for shift up and shift left

        return x_0,z_0

    def _RL_Publish(self):

        ## RL DATA
        RL_msg = RL_Data() ## Initialize RL_Data message
        
        RL_msg.k_ep = self.k_ep
        RL_msg.k_run = self.k_run
        RL_msg.error_string = self.error_str
        RL_msg.n_rollouts = self.n_rollouts


        RL_msg.policy = self.policy

        RL_msg.reward = self.reward
        RL_msg.reward_avg = self.reward_avg
        RL_msg.reward_vals = self.reward_vals

        RL_msg.vel_d = self.vel_d

        RL_msg.trialComplete_flag = self.trialComplete_flag
        self.RL_Data_Publisher.publish(RL_msg) ## Publish RL_Data message
        
        ## CONVERGENCE HISTORY
        RL_convg_msg = RL_History()
        RL_convg_msg.K_ep_list = self.K_ep_list
        RL_convg_msg.K_run_list = self.K_run_list


        RL_convg_msg.mu_1_list = self.mu_1_list
        RL_convg_msg.mu_2_list = self.mu_2_list

        RL_convg_msg.sigma_1_list = self.sigma_1_list
        RL_convg_msg.sigma_2_list = self.sigma_2_list

        RL_convg_msg.reward_list = self.reward_list

        RL_convg_msg.Kep_list_reward_avg = self.Kep_list_reward_avg
        RL_convg_msg.reward_avg_list = self.reward_avg_list

        self.RL_Convg_Publisher.publish(RL_convg_msg) ## Publish RL_Data message

        time.sleep(0.1)

    def preInit_Values(self):

        self.mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Mass")
        self.Ixx = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ixx")
        self.Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Iyy")
        self.Izz = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Izz")
        

        ## INITIALIZE STATE VALUES
        self.t = 0.0
        self.pos = [0,0,0]
        self.vel = [0,0,0]

        self.quat = [0,0,0,1]
        self.eul = [0,0,0]
        self.omega = [0,0,0]

        self.Tau = 0.0
        self.Theta_x = 0.0
        self.Theta_y = 0.0

        self.D_perp = 0.0 
        self.V_perp = 0.0
        self.V_tx = 0.0
        self.V_ty = 0.0



        self.MS_pwm = [0,0,0,0]         # Controller Motor Speeds (MS1,MS2,MS3,MS4) [PWM]
        self.MotorThrusts = [0,0,0,0]   # Controller Motor Thrusts [M1,M2,M3,M4][g]
        self.FM = [0,0,0,0]             # Controller Force/Moments (F_thrust,Mx,My,Mz) [N,N*mm]
        
        self.Policy_Trg_Action = 0.0
        self.Policy_Flip_Action = 0.0
        
        self.x_d = [0,0,0]
        self.v_d = [0,0,0]
        self.a_d = [0,0,0]

        ## INITIALIZE FLIP VALUES
        self.flip_flag = False      # Flag if model has started flip maneuver

        self.t_tr = 0.0             # [s]
        self.pos_tr = [0,0,0]       # [m]
        self.vel_tr = [0,0,0]       # [m/s]
        self.quat_tr = [0,0,0,1]    # [quat]
        self.omega_tr = [0,0,0]     # [rad/s]
        self.eul_tr = [0,0,0]       # [deg]

        self.Tau_tr = 0.0           # [s]
        self.Theta_x_tr = 0.0       # [rad/s]
        self.Theta_y_tr = 0.0       # [rad/s]
        self.D_perp_tr = 0.0        # [m]

        self.FM_tr = [0,0,0,0]      # [N,N*mm]

        self.Policy_Trg_Action_tr = 0.0
        self.Policy_Flip_Action_tr = 0.0 # [N*mm]

        self.vel_tr_mag = 0.0       # [m/s]
        self.phi_tr = 0.0           # [deg]

        ## INITIALIZE IMPACT VALUES
        self.impact_flag = False
        self.BodyContact_flag = False   # Flag if model body impacts ceiling plane

        self.t_impact = 0.0
        self.pos_impact = [0,0,0]
        self.vel_impact = [0,0,0]
        self.quat_impact = [0,0,0,1]
        self.omega_impact = [0,0,0]
        self.eul_impact = [0,0,0]

        self.impact_force_x = 0.0     # Ceiling impact force, X-dir [N]
        self.impact_force_y = 0.0     # Ceiling impact force, Y-dir [N]
        self.impact_force_z = 0.0     # Ceiling impact force, Z-dir [N]
        self.impact_magnitude = 0.0

        self.pad_connections = 0 # Number of pad connections

        ## INITIALIZE MISC VALUES
        self.V_Battery = 0.0

        ## INITIALIZE RL VALUES

        ## CONVERGENCE HISTORY
        self.K_ep_list = []
        self.K_run_list = []

        self.mu_1_list = []         # List of mu values over course of convergence
        self.mu_2_list = [] 

        self.sigma_1_list = []      # List of sigma values over course of convergence
        self.sigma_2_list = []

        self.reward_list = []       # List of reward values over course of convergence
        self.reward_avg_list = []   # List of reward averages ""
        self.Kep_list_reward_avg = []

        ## PARAM OPTIM DATA
        self.k_ep = 0                   # Episode number
        self.k_run = 0                  # Run number
        self.error_str = ""
        self.n_rollouts = 0

        self.vel_d = [0.0,0.0,0.0]      # Desired velocity for trial
        self.policy = [0.0,0.0,0.0]     # Policy sampled from Gaussian distribution

        self.reward = 0.0               # Calculated reward from run
        self.reward_avg = 0.0           # Averaged rewards over episode
        self.reward_vals = np.zeros(5)

        self.trialComplete_flag = False



    # ========================
    ##    Logging Services 
    # ========================

    def createCSV(self,logName):
        """Sends service to CF_DataConverter to create CSV log file 

        Args:
            filePath (string): Send full path and file name to write to
        """      

        ## CREATE SERVICE REQUEST MSG
        srv = Logging_CMDRequest() 
        srv.filePath = os.path.join(self.logDir,logName)
        srv.Logging_CMD = 0

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,Logging_CMD)

    def startLogging(self,logName):
        """Start logging values to the current CSV file
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = Logging_CMDRequest()
        srv.filePath = os.path.join(self.logDir,logName)
        srv.Logging_CMD = 1

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,Logging_CMD)

    def capLogging(self,logName):
        """Cap logging values with Flight, Flip, and Impact conditions and stop continuous logging
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = Logging_CMDRequest()
        srv.filePath = os.path.join(self.logDir,logName)
        srv.Logging_CMD = 2
        srv.error_string = self.error_str # String for why logging was capped
        
        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,Logging_CMD)


    # ============================
    ##   Publishers/Subscribers 
    # ============================
    def clockCallback(self,msg):
        
        if rospy.get_param('/DATA_TYPE') == "SIM":
            self.t = msg.clock.to_sec()

    def SAR_StateDataCallback(self,StateData_msg):

        if rospy.get_param('/DATA_TYPE') == "EXP":
            self.t = StateData_msg.header.stamp.to_sec()

        self.pos = np.round([StateData_msg.Pose.position.x,
                             StateData_msg.Pose.position.y,
                             StateData_msg.Pose.position.z],3)

        self.vel = np.round([StateData_msg.Twist.linear.x,
                             StateData_msg.Twist.linear.y,
                             StateData_msg.Twist.linear.z],3)

        self.eul = np.round([StateData_msg.Eul.x,
                             StateData_msg.Eul.y,
                             StateData_msg.Eul.z],3)

        self.omega = np.round([StateData_msg.Twist.angular.x,
                               StateData_msg.Twist.angular.y,
                               StateData_msg.Twist.angular.z],3)

        self.quat = np.round([StateData_msg.Pose.orientation.x,
                              StateData_msg.Pose.orientation.y,
                              StateData_msg.Pose.orientation.z,
                              StateData_msg.Pose.orientation.w],3)



        ## VISUAL STATES
        self.Tau = np.round(StateData_msg.Tau,3)
        self.Theta_x = np.round(StateData_msg.Theta_x,3)
        self.Theta_y = np.round(StateData_msg.Theta_y,3)

        ## PLANE RELATIVE STATES
        self.D_perp = np.round(StateData_msg.D_perp,3)
        self.V_perp = np.round(StateData_msg.V_perp,3)
        self.V_tx = np.round(StateData_msg.V_tx,3)
        self.V_ty = np.round(StateData_msg.V_ty,3)
       
        self.t_prev = self.t # Save t value for next callback iteration

    def SAR_TriggerDataCallback(self,TriggerData_msg):

        ## FLIP FLAG
        self.flip_flag = TriggerData_msg.flip_flag

        if TriggerData_msg.flip_flag == True:

            ## FLIP TRIGGERING CONDITIONS
            self.pos_tr = np.round([TriggerData_msg.Pose_tr.position.x,
                                    TriggerData_msg.Pose_tr.position.y,
                                    TriggerData_msg.Pose_tr.position.z],3)

            self.vel_tr = np.round([TriggerData_msg.Twist_tr.linear.x,
                                    TriggerData_msg.Twist_tr.linear.y,
                                    TriggerData_msg.Twist_tr.linear.z],3)

            self.eul_tr = np.round([TriggerData_msg.Eul_tr.x,
                                    TriggerData_msg.Eul_tr.y,
                                    TriggerData_msg.Eul_tr.z],3)

            self.omega_tr = np.round([TriggerData_msg.Twist_tr.angular.x,
                                    TriggerData_msg.Twist_tr.angular.y,
                                    TriggerData_msg.Twist_tr.angular.z],3)
            
            self.vel_tr_mag = np.sqrt(self.vel_tr[0]**2 + self.vel_tr[2]**2)
            self.phi_tr = np.rad2deg(np.arctan2(self.vel_tr[2],self.vel_tr[0]))

            ## POLICY TRIGGERING VALUES
            self.Tau_tr = TriggerData_msg.Tau_tr
            self.Theta_x_tr = TriggerData_msg.Theta_x_tr
            self.Theta_y_tr = TriggerData_msg.Theta_y_tr
            self.D_perp_tr = TriggerData_msg.D_perp_tr

            ## POLICY ACTIONS
            self.Policy_Trg_Action_tr = TriggerData_msg.Policy_Trg_Action_tr
            self.Policy_Flip_Action_tr = TriggerData_msg.Policy_Flip_Action_tr


    def SAR_ImpactDataCallback(self,ImpactData_msg):

        if rospy.get_param('/DATA_TYPE') == "SIM": ## Impact values only good in simulation

            ## IMPACT FLAGS
            self.impact_flag = ImpactData_msg.impact_flag
            self.BodyContact_flag = ImpactData_msg.BodyContact_flag
            self.pad_connections = ImpactData_msg.Pad_Connections


            ## IMPACT CONDITIONS
            self.pos_impact = np.round([ImpactData_msg.Pose_impact.position.x,
                                        ImpactData_msg.Pose_impact.position.y,
                                        ImpactData_msg.Pose_impact.position.z],3)

            self.vel_impact = np.round([ImpactData_msg.Twist_impact.linear.x,
                                        ImpactData_msg.Twist_impact.linear.y,
                                        ImpactData_msg.Twist_impact.linear.z],3)

            self.eul_impact = np.round([ImpactData_msg.Eul_impact.x,
                                        ImpactData_msg.Eul_impact.y,
                                        ImpactData_msg.Eul_impact.z],3)

            self.omega_impact = np.round([ImpactData_msg.Twist_impact.angular.x,
                                          ImpactData_msg.Twist_impact.angular.y,
                                          ImpactData_msg.Twist_impact.angular.z],3)
            
            self.Rot_Sum_impact = ImpactData_msg.Rot_Sum


    def SAR_MiscDataCallback(self,MiscData_msg):        

        self.V_Battery = np.round(MiscData_msg.battery_voltage,4)

        self.Plane_Angle = MiscData_msg.Plane_Angle
        self.Plane_Angle_rad = np.deg2rad(self.Plane_Angle)
        self.Plane_Pos = [
            MiscData_msg.Plane_Pos.x,
            MiscData_msg.Plane_Pos.y,
            MiscData_msg.Plane_Pos.z,
        ]


    def userInput(self,input_string,dataType=float):
        """Processes user input and return values as either indiviual value or list

        Args:
            input_string (string): String received from user
            dataType (dataType, optional): Datatype to parse string to. Defaults to float.

        Returns:
            vals: Values parsed by ','. If multiple values then return list
        """        

        while True:
            try:
                vals = [dataType(i) for i in input(input_string).split(',')]
            except:
                continue
        
            ## RETURN MULTIPLE VALUES IF MORE THAN ONE
            if len(vals) == 1:
                return vals[0]
            else:
                return vals


if __name__ == "__main__":

    env = SAR_Base_Interface()
    rospy.spin()


