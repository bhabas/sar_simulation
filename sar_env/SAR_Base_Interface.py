#!/usr/bin/env python3
import numpy as np
import os
import rospy
import getpass
import time
import warnings 

## ROS MESSAGES AND SERVICES
from sar_msgs.msg import SAR_StateData,SAR_TriggerData,SAR_ImpactData,SAR_MiscData
from sar_msgs.srv import Logging_CMD,Logging_CMDRequest
from sar_msgs.srv import CTRL_Cmd_srv,CTRL_Cmd_srvRequest
from sar_msgs.msg import RL_Data,RL_History

from rosgraph_msgs.msg import Clock

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = "\033[34m"  # Blue text
RESET = "\033[0m"  # Reset to default color

class SAR_Base_Interface():

    def __init__(self,Exp_Flag=False):
        
        if not Exp_Flag:   
            os.system("roslaunch sar_launch Load_Params.launch")
            rospy.init_node("SAR_Env_Node")

        ## SAR PARAMETERS
        self.SAR_Type = rospy.get_param('/SAR_SETTINGS/SAR_Type')
        self.SAR_Config = rospy.get_param('/SAR_SETTINGS/SAR_Config')

        self.Pos_0 = [0.0, 0.0, 0.4]      # Default hover position [m]


        ## INERTIAL PARAMETERS
        self.Ref_Mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Mass")
        self.Ref_Ixx = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Ixx")
        self.Ref_Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Iyy")
        self.Ref_Izz = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Izz")

        ## GEOMETRIC PARAMETERS
        self.Forward_Reach = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Forward_Reach")
        self.Leg_Length = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Leg_Length")
        self.Leg_Angle = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Leg_Angle")
        self.Prop_Front = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Prop_Front")
        self.Prop_Rear = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Prop_Rear")

        ## EFFECTIVE-GEOEMTRIC PARAMETERS
        self.L_eff = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/L_eff")
        self.Gamma_eff = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Gamma_eff")
        self.Lx_eff = self.L_eff*np.sin(np.radians(self.Gamma_eff))
        self.Lz_eff = self.L_eff*np.cos(np.radians(self.Gamma_eff))
        self.Collision_Radius = max(self.L_eff,self.Forward_Reach)

        ## SYSTEM AND FLIGHT PARAMETERS
        self.Thrust_max = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Thrust_max")
        self.TrajAcc_Max = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/TrajAcc_Max")
        self.Ang_Acc_max = (9.81*self.Thrust_max*1e-3*self.Prop_Front[0])*2/self.Ref_Iyy
        self.setAngAcc_range([-self.Ang_Acc_max, self.Ang_Acc_max])
        
        self.Beta_Min_deg = -(self.Gamma_eff + np.degrees(np.arctan2(self.Forward_Reach-self.Lx_eff,self.Lz_eff)))
        self.Phi_impact_P_B_Min_deg = -self.Beta_Min_deg - self.Gamma_eff + 90

        # self.Phi_impact_B_O_Min_deg = self.Beta_Min_deg + self.Gamma_eff + self.Plane_Angle_deg - 90 


        ## CAM PARAMETERS
        self.Cam_Config = rospy.get_param('/CAM_SETTINGS/Cam_Config')
        self.Cam_Active = rospy.get_param('/CAM_SETTINGS/Cam_Active')
        

        ## PLANE PARAMETERS
        self.Plane_Type = rospy.get_param('/PLANE_SETTINGS/Plane_Type')
        self.Plane_Config = rospy.get_param('/PLANE_SETTINGS/Plane_Config')
        

        ## LOGGING PARAMETERS
        self.Username = getpass.getuser()
        self.Log_Dir =  f"/home/{self.Username}/catkin_ws/src/sar_simulation/sar_logging/local_logs"
        self.Log_Name = "TestLog.csv"
        self.Error_Str = "No_Debug_Data"

        print(f"{GREEN}")
        print("=============================================")
        print("       SAR Base Interface Initialized        ")
        print("=============================================")

        print(f"SAR Type: {self.SAR_Type} -- SAR Config: {self.SAR_Config}\n")
        print(f"Leg Length: {self.Leg_Length:.3f} m \t Leg Angle: {self.Leg_Angle:.1f} deg")
        print(f"L_eff: {self.L_eff:.3f} m \t\t Gamma_eff: {self.Gamma_eff:.1f} deg\n")
        print(f"Phi_impact_P_B_Min: {self.Phi_impact_P_B_Min_deg:.1f} deg\n")

        print(f"Thrust_max: {self.Thrust_max:.0f} g")
        print(f"Ang_Acc_Max: {self.Ang_Acc_max:.0f} rad/s^2")
        print(f"TrajAcc_Max: [{self.TrajAcc_Max[0]:.1f}, {self.TrajAcc_Max[1]:.1f}, {self.TrajAcc_Max[2]:.1f}]  m/s^2") 

        print(f"{RESET}")



        ## SAR DATA SUBSCRIBERS 
        # NOTE: Queue sizes=1 so that we are always looking at the most current data and 
        #       not data at back of a queue waiting to be processed by callbacks
        rospy.Subscriber("/clock",Clock,self.clockCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/StateData",SAR_StateData,self._SAR_StateDataCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/TriggerData",SAR_TriggerData,self._SAR_TriggerDataCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/ImpactData",SAR_ImpactData,self._SAR_ImpactDataCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/MiscData",SAR_MiscData,self._SAR_MiscDataCallback,queue_size=1)

        ## RL DATA PUBLISHERS
        self.RL_Data_Pub = rospy.Publisher('/RL/Data',RL_Data,queue_size=10)
        self.RL_History_Pub = rospy.Publisher('/RL/History',RL_History,queue_size=10)

    def _getTime(self):
        """Returns current known time.

        Returns:
            float: Current known time.
        """        
        
        return self.t

    def sendCmd(self,action,cmd_vals=[0,0,0],cmd_flag=1):
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
            'dOmega':7,
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

    def setAngAcc_range(self,Ang_Acc_range):
        """Sets the range of allowable angular accelerations for the model

        Args:
            Ang_Acc_range (list): List of min/max angular accelerations [min,max]
        """        

        if max(abs(i) for i in Ang_Acc_range) > self.Ang_Acc_max:
            rospy.logwarn(f"Angular Acceleration range exceeds max value of {self.Ang_Acc_max:.0f} deg/s^2")
            rospy.logwarn(f"Setting Angular Acceleration range to max value")
            self.Ang_Acc_range = [-self.Ang_Acc_max,self.Ang_Acc_max]
        else:
            self.Ang_Acc_range = Ang_Acc_range
                

    def startPos_VelTraj(self,x_impact,V_d,accel_d=None,Tau_0=0.5):
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
            accel_d = self.TrajAcc_Max
    
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
    
    def setPlanePose(self,Position=[0,0,2.0],Plane_Angle=180):
        
        self.sendCmd("GZ_Plane_Pose",Position,Plane_Angle)

    def _RL_Publish(self):

        ## RL DATA
        RL_msg = RL_Data() ## Initialize RL_Data message
        
        RL_msg.K_ep = self.K_ep
        RL_msg.K_run = self.K_run
        RL_msg.error_string = self.Error_Str
        RL_msg.n_rollouts = self.n_rollouts


        RL_msg.policy = self.policy

        RL_msg.reward = self.reward
        RL_msg.reward_avg = self.reward_avg
        RL_msg.reward_vals = self.reward_vals

        RL_msg.vel_d = self.vel_d

        RL_msg.trialComplete_flag = self.trialComplete_flag
        self.RL_Data_Pub.publish(RL_msg) ## Publish RL_Data message
        
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

        self.RL_History_Pub.publish(RL_convg_msg) ## Publish RL_Data message

        time.sleep(0.1)



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
        srv.filePath = os.path.join(self.Log_Dir,logName)
        srv.Logging_CMD = 0

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,Logging_CMD)

    def startLogging(self,logName):
        """Start logging values to the current CSV file
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = Logging_CMDRequest()
        srv.filePath = os.path.join(self.Log_Dir,logName)
        srv.Logging_CMD = 1

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,Logging_CMD)

    def capLogging(self,logName):
        """Cap logging values with Flight, Rot, and Impact conditions and stop continuous logging
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = Logging_CMDRequest()
        srv.filePath = os.path.join(self.Log_Dir,logName)
        srv.Logging_CMD = 2
        srv.error_string = self.Error_Str # String for why logging was capped
        
        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,Logging_CMD)


    # ============================
    ##   Publishers/Subscribers 
    # ============================
    def clockCallback(self,msg):
        
        if rospy.get_param('/DATA_TYPE') == "SIM":
            self.t = msg.clock.to_sec()

    def _SAR_StateDataCallback(self,StateData_msg):

        if rospy.get_param('/DATA_TYPE') == "EXP":
            self.t = StateData_msg.Time.data.to_sec()

        ## STATES WRT ORIGIN
        self.r_B_O = np.round([StateData_msg.Pose_B_O.position.x,
                                StateData_msg.Pose_B_O.position.y,
                                StateData_msg.Pose_B_O.position.z],3)
        self.V_B_O = np.round([StateData_msg.Twist_B_O.linear.x,
                                StateData_msg.Twist_B_O.linear.y,
                                StateData_msg.Twist_B_O.linear.z],3)
        
        self.eul_B_O = np.round([StateData_msg.Eul_B_O.x,
                                    StateData_msg.Eul_B_O.y,
                                    StateData_msg.Eul_B_O.z],3)
        self.omega_B_O = np.round([StateData_msg.Twist_B_O.angular.x,
                                    StateData_msg.Twist_B_O.angular.y,
                                    StateData_msg.Twist_B_O.angular.z],3)
        
        ## STATES WRT PLANE
        self.r_P_B = np.round([StateData_msg.Pose_P_B.position.x,
                                StateData_msg.Pose_P_B.position.y,
                                StateData_msg.Pose_P_B.position.z],3)
        
        self.Eul_P_B = np.round([StateData_msg.Eul_P_B.x,
                                    StateData_msg.Eul_P_B.y,
                                    StateData_msg.Eul_P_B.z],3)
        
        self.V_B_P = np.round([StateData_msg.Twist_B_P.linear.x,
                                StateData_msg.Twist_B_P.linear.y,
                                StateData_msg.Twist_B_P.linear.z],3)
        
        self.Omega_B_P = np.round([StateData_msg.Twist_B_P.angular.x,
                                    StateData_msg.Twist_B_P.angular.y,
                                    StateData_msg.Twist_B_P.angular.z],3)
        
        self.Vel_mag_B_P = np.round(StateData_msg.Vel_mag_B_P,3)
        self.Vel_angle_B_P = np.round(StateData_msg.Vel_angle_B_P,3)
        self.D_perp = np.round(StateData_msg.D_perp,3)
        self.D_perp_CR = np.round(StateData_msg.D_perp_CR,3)

        ## OPTICAL FLOW STATES
        self.Theta_x = np.round(StateData_msg.Optical_Flow.x,3)
        self.Theta_y = np.round(StateData_msg.Optical_Flow.y,3)
        self.Tau = np.round(StateData_msg.Optical_Flow.z,3)
        self.Tau_CR = np.round(StateData_msg.Tau_CR,3)
        
        self.t_prev = self.t # Save t value for next callback iteration

    def _SAR_TriggerDataCallback(self,TriggerData_msg):

        ## TRIGGER FLAG
        self.Trg_Flag = TriggerData_msg.Trg_Flag

        if TriggerData_msg.Trg_Flag == True:

            ## POLICY TRIGGERING CONDITIONS
            self.t_trg = TriggerData_msg.Time_trg.data.to_sec()

            ## STATES WRT ORIGIN
            self.r_B_O_trg = np.round([TriggerData_msg.Pose_B_O_trg.position.x,
                                        TriggerData_msg.Pose_B_O_trg.position.y,
                                        TriggerData_msg.Pose_B_O_trg.position.z],3)
            
            self.V_B_O_trg = np.round([TriggerData_msg.Twist_B_O_trg.linear.x,
                                        TriggerData_msg.Twist_B_O_trg.linear.y,
                                        TriggerData_msg.Twist_B_O_trg.linear.z],3)
            
            self.Eul_B_O_trg = np.round([TriggerData_msg.Eul_B_O_trg.x,
                                        TriggerData_msg.Eul_B_O_trg.y,
                                        TriggerData_msg.Eul_B_O_trg.z],3)
            
            self.Omega_B_O_trg = np.round([TriggerData_msg.Twist_B_O_trg.angular.x,
                                            TriggerData_msg.Twist_B_O_trg.angular.y,
                                            TriggerData_msg.Twist_B_O_trg.angular.z],3)

            ## STATES WRT PLANE
            self.r_P_B_trg = np.round([TriggerData_msg.Pose_P_B_trg.position.x,
                                        TriggerData_msg.Pose_P_B_trg.position.y,
                                        TriggerData_msg.Pose_P_B_trg.position.z],3)
            
            self.Eul_P_B_trg = np.round([TriggerData_msg.Eul_P_B_trg.x,
                                        TriggerData_msg.Eul_P_B_trg.y,
                                        TriggerData_msg.Eul_P_B_trg.z],3)
            
            self.V_B_P_trg = np.round([TriggerData_msg.Twist_B_P_trg.linear.x,  
                                        TriggerData_msg.Twist_B_P_trg.linear.y,
                                        TriggerData_msg.Twist_B_P_trg.linear.z],3)
            
            self.Omega_B_P_trg = np.round([TriggerData_msg.Twist_B_P_trg.angular.x,
                                            TriggerData_msg.Twist_B_P_trg.angular.y,
                                            TriggerData_msg.Twist_B_P_trg.angular.z],3)
            
            self.D_perp_trg = np.round(TriggerData_msg.D_perp_trg,3)
            self.D_perp_CR_trg = np.round(TriggerData_msg.D_perp_CR_trg,3)

            ## OPTICAL FLOW STATES
            self.Theta_x_trg = np.round(TriggerData_msg.Optical_Flow_trg.x,3)
            self.Theta_y_trg = np.round(TriggerData_msg.Optical_Flow_trg.y,3)
            self.Tau_trg = np.round(TriggerData_msg.Optical_Flow_trg.z,3)
            self.Tau_CR_trg = np.round(TriggerData_msg.Tau_CR_trg,3)

            ## POLICY TRIGGERING CONDITIONS
            self.Policy_Trg_Action_trg = np.round(TriggerData_msg.Policy_Trg_Action_trg,3)
            self.Policy_Rot_Action_trg = np.round(TriggerData_msg.Policy_Rot_Action_trg,3)

    def _SAR_ImpactDataCallback(self,ImpactData_msg):

        self.Impact_Flag = ImpactData_msg.Impact_Flag

        if ImpactData_msg.Impact_Flag_OB == True:
            self.Impact_Flag_OB = ImpactData_msg.Impact_Flag_OB

            self.t_impact_OB = ImpactData_msg.Time_impact_OB.data.to_sec()

            self.r_B_O_impact_OB = np.round([ImpactData_msg.Pose_B_O_impact_OB.position.x,
                                            ImpactData_msg.Pose_B_O_impact_OB.position.y,
                                            ImpactData_msg.Pose_B_O_impact_OB.position.z],3)
            
            self.Eul_B_O_impact_OB = np.round([ImpactData_msg.Eul_B_O_impact_OB.x,
                                                ImpactData_msg.Eul_B_O_impact_OB.y,
                                                ImpactData_msg.Eul_B_O_impact_OB.z],3)
            
            self.V_B_P_impact_OB = np.round([ImpactData_msg.Twist_B_P_impact_OB.linear.x,
                                            ImpactData_msg.Twist_B_P_impact_OB.linear.y,
                                            ImpactData_msg.Twist_B_P_impact_OB.linear.z],3)
            
            self.Omega_B_P_impact_OB = np.round([ImpactData_msg.Twist_B_P_impact_OB.angular.x,
                                                ImpactData_msg.Twist_B_P_impact_OB.angular.y,
                                                ImpactData_msg.Twist_B_P_impact_OB.angular.z],3)
            
            self.Eul_P_B_impact_OB = np.round([ImpactData_msg.Eul_P_B_impact_OB.x,
                                            ImpactData_msg.Eul_P_B_impact_OB.y,
                                            ImpactData_msg.Eul_P_B_impact_OB.z],3)
            
            self.Accel_B_O_Mag_impact_OB = np.round(ImpactData_msg.Accel_B_O_Mag_impact_OB,3)

        if ImpactData_msg.Impact_Flag_Ext == True:
            self.Impact_Flag_Ext = ImpactData_msg.Impact_Flag_Ext
            
            self.t_impact_Ext = ImpactData_msg.Time_impact_Ext.data.to_sec()

            self.r_B_O_impact_Ext = np.round([ImpactData_msg.Pose_B_O_impact_Ext.position.x,
                                            ImpactData_msg.Pose_B_O_impact_Ext.position.y,
                                            ImpactData_msg.Pose_B_O_impact_Ext.position.z],3)
            
            self.Eul_B_O_impact_Ext = np.round([ImpactData_msg.Eul_B_O_impact_Ext.x,
                                                ImpactData_msg.Eul_B_O_impact_Ext.y,
                                                ImpactData_msg.Eul_B_O_impact_Ext.z],3)
            
            self.V_B_P_impact_Ext = np.round([ImpactData_msg.Twist_B_P_impact_Ext.linear.x,
                                            ImpactData_msg.Twist_B_P_impact_Ext.linear.y,
                                            ImpactData_msg.Twist_B_P_impact_Ext.linear.z],3)
            
            self.Omega_B_P_impact_Ext = np.round([ImpactData_msg.Twist_B_P_impact_Ext.angular.x,
                                                ImpactData_msg.Twist_B_P_impact_Ext.angular.y,
                                                ImpactData_msg.Twist_B_P_impact_Ext.angular.z],3)
            
            self.Eul_P_B_impact_Ext = np.round([ImpactData_msg.Eul_P_B_impact_Ext.x,
                                            ImpactData_msg.Eul_P_B_impact_Ext.y,
                                            ImpactData_msg.Eul_P_B_impact_Ext.z],3)
            
            self.Rot_Sum_impact_Ext = np.round(ImpactData_msg.Rot_Sum_impact_Ext,3)

        
        ## IMPACT FORCES
        self.Force_impact_x = np.round(ImpactData_msg.Force_impact.x,3)
        self.Force_impact_y = np.round(ImpactData_msg.Force_impact.y,3)
        self.Force_impact_z = np.round(ImpactData_msg.Force_impact.z,3)
        self.Impact_Magnitude = np.round(ImpactData_msg.Impact_Magnitude,3)

        ## PAD CONNECTIONS
        self.Pad_Connections = ImpactData_msg.Pad_Connections
        self.Pad1_Contact = ImpactData_msg.Pad1_Contact
        self.Pad2_Contact = ImpactData_msg.Pad2_Contact
        self.Pad3_Contact = ImpactData_msg.Pad3_Contact
        self.Pad4_Contact = ImpactData_msg.Pad4_Contact
            



        

    def _SAR_MiscDataCallback(self,MiscData_msg):        

        self.V_Battery = np.round(MiscData_msg.battery_voltage,4)

        self.Plane_Angle_deg = MiscData_msg.Plane_Angle
        self.Plane_Angle_rad = np.deg2rad(self.Plane_Angle_deg)
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

    # ============================
    ##   Rotation Matrices 
    # ============================
    def R_BW(self,vec,phi):

        R_BW = np.array([
            [ np.cos(phi), np.sin(phi)],
            [-np.sin(phi), np.cos(phi)],
        ])

        return R_BW.dot(vec)
    
    def R_WP(self,vec,theta):

        R_WP = np.array([
            [ np.cos(theta), 0, -np.sin(theta)],
            [     0,         1,       0      ],
            [ np.sin(theta), 0,  np.cos(theta)]
        ])

        return R_WP.dot(vec)
    
    def R_PW(self,vec,theta):

        R_PW = np.array([
            [ np.cos(theta), 0, np.sin(theta)],
            [     0,         1,       0      ],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

        return R_PW.dot(vec)
    
    def R_PC1(self,vec,Beta1):

        R_PC1 = np.array([
            [ np.cos(Beta1),-np.sin(Beta1)],
            [ np.sin(Beta1), np.cos(Beta1)]
        ])

        return R_PC1.dot(vec)
    
    def R_C1P(self,vec,Beta1):

        R_C1P = np.array([
            [ np.cos(Beta1), np.sin(Beta1)],
            [-np.sin(Beta1), np.cos(Beta1)]
        ])

        return R_C1P.dot(vec)
    
    def R_C1B(self,vec,gamma_rad):

        R_C1B = np.array([
            [ np.sin(gamma_rad), np.cos(gamma_rad)],
            [-np.cos(gamma_rad), np.sin(gamma_rad)],
        ])

        return R_C1B.dot(vec)

    def R_PC2(self,vec,Beta2):

        R_PC2 = np.array([
            [ np.cos(Beta2), np.sin(Beta2)],
            [-np.sin(Beta2), np.cos(Beta2)]
        ])

        return R_PC2.dot(vec)
    
    def R_C2P(self,vec,Beta2):

        R_C2P = np.array([
            [ np.cos(Beta2), np.sin(Beta2)],
            [-np.sin(Beta2), np.cos(Beta2)],
        ])

        return R_C2P.dot(vec)

    def R_C2B(self,vec,gamma_rad):

        R_C2B = np.array([
            [-np.sin(gamma_rad), np.cos(gamma_rad)],
            [-np.cos(gamma_rad),-np.sin(gamma_rad)],
        ])

        return R_C2B.dot(vec)


if __name__ == "__main__":

    env = SAR_Base_Interface()
    rospy.spin()


