#!/usr/bin/env python3
import numpy as np
import os
import rospy
import getpass
import time
import warnings 
import yaml
import rospkg

## ROS MESSAGES AND SERVICES
from sar_msgs.msg import SAR_StateData,SAR_TriggerData,SAR_ImpactData,SAR_MiscData
from sar_msgs.srv import Logging_CMD,Logging_CMDRequest
from sar_msgs.srv import CTRL_Cmd_srv,CTRL_Cmd_srvRequest
from sar_msgs.msg import RL_Data,RL_History

from rosgraph_msgs.msg import Clock

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = "\033[34m"  
RESET = "\033[0m"  # Reset to default color

class SAR_Base_Interface():

    def __init__(self,Experiment_Setup=False):

        ## LOGGING PARAMETERS
        self.BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))
        self.Log_Dir =  f"{self.BASE_PATH}/sar_logging/local_logs"
        self.Log_Name = "TestLog.csv"
        self.Error_Str = "No_Debug_Data"
        
        if not Experiment_Setup:   
            rospy.init_node("SAR_Env_Node")
            self.loadBaseParams()
            self._preInit()
            
        else:
            self.loadBaseParams()
            self._preInit()


        print(f"{GREEN}")
        print("=============================================")
        print("       SAR Base Interface Initialized        ")
        print("=============================================")

        print(f"SAR Type: {self.SAR_Type} -- SAR Config: {self.SAR_Config}\n")
        print(f"Leg Length: {self.Leg_Length:.3f} m \t Leg Angle: {self.Leg_Angle:.1f} deg")
        print(f"L_eff: {self.L_eff:.3f} m \t\t Gamma_eff: {self.Gamma_eff:.1f} deg\n")
        print(f"Phi_impact_P_B_Min: {self.Phi_P_B_impact_Min_deg:.1f} deg\n")

        print(f"Thrust_max: {self.Thrust_max:.0f} g")
        print(f"Ang_Acc_Max: {self.Ang_Acc_max:.0f} rad/s^2")
        print(f"TrajAcc_Max: [{self.TrajAcc_Max[0]:.1f}, {self.TrajAcc_Max[1]:.1f}, {self.TrajAcc_Max[2]:.1f}]  m/s^2") 

        print(f"{RESET}")



        ## SAR DATA SUBSCRIBERS 
        # NOTE: Queue sizes=1 so that we are always looking at the most current data and 
        #       not data at back of a queue waiting to be processed by callbacks
        rospy.Subscriber("/clock",Clock,self._clockCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/StateData",SAR_StateData,self._SAR_StateDataCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/TriggerData",SAR_TriggerData,self._SAR_TriggerDataCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/ImpactData",SAR_ImpactData,self._SAR_ImpactDataCallback,queue_size=1)
        rospy.Subscriber("/SAR_DC/MiscData",SAR_MiscData,self._SAR_MiscDataCallback,queue_size=1)

        ## RL DATA PUBLISHERS
        self.RL_Data_Pub = rospy.Publisher("/RL/Data",RL_Data,queue_size=10)
        self.RL_History_Pub = rospy.Publisher("/RL/History",RL_History,queue_size=10)

    def loadBaseParams(self):

        ## LOAD BASE PARAMETERS
        param_path_list = [f"{self.BASE_PATH}/sar_config/Base_Settings.yaml",
                     f"{self.BASE_PATH}/sar_config/Model_Types.yaml",
                     f"{self.BASE_PATH}/sar_config/Cam_Types.yaml",
        ]

        for path in param_path_list:

            with open(path, 'r') as file:
                loaded_parameters = yaml.safe_load(file)

            # Load parameters into the ROS Parameter Server
            for param_name, param_value in loaded_parameters.items():
                rospy.set_param(param_name, param_value)

        

        ## SAR PARAMETERS
        self.SAR_Type = rospy.get_param('/SAR_SETTINGS/SAR_Type')
        self.SAR_Config = rospy.get_param('/SAR_SETTINGS/SAR_Config')
        self.Policy_Type = rospy.get_param('/SAR_SETTINGS/Policy_Type')

        ## INERTIAL PARAMETERS
        self.Ref_Mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Mass")
        self.Ref_Ixx = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Ixx")
        self.Ref_Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Iyy")
        self.Ref_Izz = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Ref_Izz")

        self.Base_Mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Base_Mass")
        self.Base_Ixx = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Base_Ixx")
        self.Base_Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Base_Iyy")
        self.Base_Izz = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Base_Izz")

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
        self.Collision_Radius = self.L_eff

        ## SYSTEM AND FLIGHT PARAMETERS
        self.TrajAcc_Max = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/TrajAcc_Max")
        self.TrajJerk_Max = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/TrajJerk_Max")
        self.Tau_up = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Tau_up")
        self.Tau_down = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Tau_down")
        self.Thrust_max = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/Thrust_max")
        self.C_tf = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/System_Params/C_tf")
        self.Ang_Acc_max = (9.81*self.Thrust_max*1e-3*self.Prop_Front[0])*2/self.Ref_Iyy
        self.setAngAcc_range([-self.Ang_Acc_max, self.Ang_Acc_max])
        
        self.Beta_Min_deg = -(self.Gamma_eff + np.degrees(np.arctan2(self.Forward_Reach-self.Lx_eff,self.Lz_eff)))
        self.Phi_P_B_impact_Min_deg = -self.Beta_Min_deg - self.Gamma_eff + 90

        ## CAM PARAMETERS
        self.Cam_Config = rospy.get_param('/CAM_SETTINGS/Cam_Config')
        self.Cam_Active = rospy.get_param('/CAM_SETTINGS/Cam_Active')
        

        ## PLANE PARAMETERS
        self.Plane_Type = rospy.get_param('/PLANE_SETTINGS/Plane_Type')
        self.Plane_Config = rospy.get_param('/PLANE_SETTINGS/Plane_Config')
        self.Plane_Pos_x_init = rospy.get_param('/PLANE_SETTINGS/Pos_X_init')
        self.Plane_Pos_y_init = rospy.get_param('/PLANE_SETTINGS/Pos_Y_init')
        self.Plane_Pos_z_init = rospy.get_param('/PLANE_SETTINGS/Pos_Z_init')
        self.Plane_Angle_deg_init = rospy.get_param('/PLANE_SETTINGS/Plane_Angle_init')

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

        ## CREATE SERVICE REQUEST MSG
        srv = CTRL_Cmd_srvRequest() 
        
        srv.cmd_type = self.cmd_dict[action]
        srv.cmd_vals.x = cmd_vals[0]
        srv.cmd_vals.y = cmd_vals[1]
        srv.cmd_vals.z = cmd_vals[2]
        srv.cmd_flag = cmd_flag
        srv.cmd_rx = True

        self.callService('/SAR_DC/CMD_Input',srv,CTRL_Cmd_srv)    

    def callService(self,srv_addr,srv_msg,srv_type,num_retries=5):

        ## CHECK THAT SERVICE IS AVAILABLE
        try:
            rospy.wait_for_service(srv_addr, timeout=1)

        except rospy.ROSException as e:
            rospy.logerr(f"[WARNING] Service '{srv_addr}' not available: {e}")
            return None
            
        ## CALL SERVICE AND RETURN RESPONSE
        service_proxy = rospy.ServiceProxy(srv_addr, srv_type)
        for retry in range(num_retries):
            try:
                if srv_msg is None:
                    response = service_proxy()
                else:
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
                
    def startPos_ImpactTraj(self,V_B_P,Acc=None,Tau_CR_start=1.0):

        ## DESIRED RELATIVE VELOCITY VALUES
        V_tx,_,V_perp = V_B_P

        ## CALCULATE STARTING TAU VALUE
        Tau_Body_start = (Tau_CR_start + self.Collision_Radius/V_perp) # Tau read by body

        ## CALC STARTING POSITION IN GLOBAL COORDS
        # (Derivation: Research_Notes_Book_3.pdf (9/17/23))
        r_P_O = np.array(self.r_P_O)                        # Plane Position wrt to Origin - {X_W,Y_W,Z_W}
        r_P_B = np.array([(Tau_CR_start)*V_tx,
                          0,
                          (Tau_Body_start)*V_perp])             # Body Position wrt to Plane - {t_x,t_y,n_p}
        r_B_O = r_P_O - self.R_PW(r_P_B,self.Plane_Angle_rad)   # Body Position wrt to Origin - {X_W,Y_W,Z_W}



        ## DESIRED GLOBAL VELOCITY VALUES
        V_B_O = self.R_PW(V_B_P,self.Plane_Angle_rad)           # Body Velocity wrt to Origin - {X_W,Y_W,Z_W}


        ## DESIRED ACCELERATION VALUES
        if Acc == None:
            Acc = self.TrajAcc_Max
    
        a_x = Acc[0]
        a_z = Acc[2]


        ## CALC OFFSET POSITIONS
        t_x = V_B_O[0]/a_x    # Time required to reach Vx
        t_z = V_B_O[2]/a_z    # Time required to reach Vz

        x_0 = r_B_O[0] - V_B_O[0]**2/(2*a_x) - V_B_O[0]*t_z     # X-position Vel reached
        y_0 = r_B_O[1]                                          # Y-position Vel reached
        z_0 = r_B_O[2] - V_B_O[2]**2/(2*a_z)                    # Z-position Vel reached    

        return [x_0,y_0,z_0]
    
    def _setPlanePose(self,Position=[0,0,2.0],Plane_Angle=180):
        
        self.sendCmd("Plane_Pose",Position,Plane_Angle)

    def _sampleFlightConditions(self,V_mag_range=[0.5,1.5],V_angle_range=[0,180]):

        ## SAMPLE V_MAG FROM UNIFORM DISTRIBUTION IN MAGNITUDE RANGE
        V_mag_Low = V_mag_range[0]
        V_mag_High = V_mag_range[1]
        V_mag = np.random.uniform(low=V_mag_Low,high=V_mag_High)

        ## CONVERT RELATIVE ANGLES TO GLOBAL ANGLE
        A1 = V_angle_range[0] - self.Plane_Angle_deg
        A2 = V_angle_range[1] - self.Plane_Angle_deg

        ## ANGLE CAPS TO ENSURE +X DIRECTION
        B1 = -90
        B2 = 90

        ## CAP ANGLES TO BE WITHIN -90 to 90 DEGREES
        A1 = np.clip(A1,B1,B2)
        A2 = np.clip(A2,B1,B2)

        ## CONVERT ANGLES BACK TO RELATIVE VALUES
        V_angle_Low = A1 + self.Plane_Angle_deg
        V_angle_High = A2 + self.Plane_Angle_deg

        ## SAMPLE RELATIVE V_ANGLE FROM A WEIGHTED SET OF UNIFORM DISTRIBUTIONS
        V_Angle_range = V_angle_High - V_angle_Low
        Dist_Num = np.random.choice([0,1,2],p=[0.1,0.8,0.1]) # Probability of sampling distribution

        if Dist_Num == 0: # Low Range
            Flight_Angle = np.random.default_rng().uniform(low=V_angle_Low, high=V_angle_Low + 0.1*V_Angle_range)
        elif Dist_Num == 1: # Medium Range
            Flight_Angle = np.random.default_rng().uniform(low=V_angle_Low + 0.1*V_Angle_range, high=V_angle_High - 0.1*V_Angle_range)
        elif Dist_Num == 2: # High Range
            Flight_Angle = np.random.default_rng().uniform(low=V_angle_High - 0.1*V_Angle_range, high=V_angle_High)
       
        return V_mag,Flight_Angle

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
    ##      Command Handlers 
    # ============================
    def handle_Ctrl_Reset(self):
        print("Reset controller to default values\n")
        self.sendCmd("GZ_StickyPads",cmd_flag=0)
        self.sendCmd("Ctrl_Reset")

    def handle_Pos_Cmd(self):
        cmd_vals = self.userInput("Set desired position values (x,y,z): ",float)
        cmd_flag = self.userInput("Pos control On/Off (1,0): ",int)
        self.sendCmd("Pos",cmd_vals,cmd_flag)

    def handle_Vel_Cmd(self):
        cmd_vals = self.userInput("Set desired velocity values (x,y,z): ",float)
        cmd_flag = self.userInput("Vel control On/Off (1,0): ",int)
        self.sendCmd("Vel",cmd_vals,cmd_flag)

    def handle_Ang_Accel(self):
        cmd_vals = self.userInput("Set desired angular acceleration values (x,y,z): ",float)
        cmd_flag = self.userInput("Ang_Accel control On/Off (1,0): ",int)
        self.sendCmd("Ang_Accel",cmd_vals,cmd_flag)

    def handle_Policy(self):
        action_vals = self.userInput("Set desired policy actions (a_Trg,a_Rot): ",float)
        scale_vals = self.userInput("Set desired a_Rot scaling (a_Rot_low,a_Rot_high): ",float)
        cmd_vals = [action_vals[0],action_vals[1],scale_vals[0]]
        cmd_flag = scale_vals[1]
        self.sendCmd("Policy",cmd_vals,cmd_flag)

    def handle_Plane_Pose(self):
        cmd_vals = self.userInput("Set desired position values (x,y,z): ",float)
        cmd_flag = self.userInput("Set desired plane angle [deg]: ",float)
        self._setPlanePose(cmd_vals,cmd_flag)

    ## ========== TRAJECTORY FUNCTIONS ==========
        
    def handle_P2P_traj(self):
        x_d = self.userInput("Desired position (x,y,z):",float)
        self.sendCmd('P2P_traj',cmd_vals=[np.nan,x_d[0],0.5],cmd_flag=0)
        self.sendCmd('P2P_traj',cmd_vals=[np.nan,x_d[1],0.5],cmd_flag=1)
        self.sendCmd('P2P_traj',cmd_vals=[np.nan,x_d[2],0.5],cmd_flag=2)
        self.sendCmd('Activate_traj',cmd_vals=[1,1,1])

    def handle_Global_Vel_traj(self):

        ## GET GLOBAL VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        ## CALC GLOBAL VELOCITIES
        Vx = V_mag*np.cos(np.radians(V_angle))
        Vy = 0
        Vz = V_mag*np.sin(np.radians(V_angle))
        V_B_O = [Vx,Vy,Vz]

        ## EXECUTE TRAJECTORY
        self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[0],self.TrajAcc_Max[0],self.TrajJerk_Max[0]],cmd_flag=0)
        self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[2],self.TrajAcc_Max[2],self.TrajJerk_Max[2]],cmd_flag=2)
        self.sendCmd('Activate_traj',cmd_vals=[1,0,1])

    def handle_Rel_Vel_traj(self):

        ## GET RELATIVE VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        ## CALC RELATIVE VELOCITIES
        V_tx = V_mag*np.cos(np.radians(V_angle))
        V_ty = 0
        V_perp = V_mag*np.sin(np.radians(V_angle))

        ## CALCULATE GLOBAL VELOCITIES
        V_B_O = self.R_PW(np.array([V_tx,V_ty,V_perp]),self.Plane_Angle_rad)

        ## EXECUTE TRAJECTORY
        self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[0],self.TrajAcc_Max[0],self.TrajJerk_Max[0]],cmd_flag=0)
        self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[2],self.TrajAcc_Max[2],self.TrajJerk_Max[2]],cmd_flag=2)
        self.sendCmd('Activate_traj',cmd_vals=[1,0,1])

    def handle_Impact_traj(self):

        ## GET GLOBAL VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        V_x = V_mag*np.cos(np.radians(V_angle))
        V_y = 0
        V_z = V_mag*np.sin(np.radians(V_angle))
        V_B_O = np.array([V_x,V_y,V_z])


        ## CALC RELATIVE VELOCITIES
        V_B_P = self.R_WP(V_B_O,self.Plane_Angle_rad)

        ## POS VELOCITY CONDITIONS MET
        r_B_O = self.startPos_ImpactTraj(V_B_P,Acc=None)

        ## APPROVE START POSITION
        print(YELLOW,f"Start Position: ({r_B_O[0]:.2f},{self.r_B_O[1]:.2f},{r_B_O[2]:.2f})",RESET)
        str_input = self.userInput("Approve start position (y/n): ",str)
        if str_input == 'y':
            self.sendCmd('P2P_traj',cmd_vals=[np.nan,r_B_O[0],0.5],cmd_flag=0)
            self.sendCmd('P2P_traj',cmd_vals=[np.nan,r_B_O[1],0.5],cmd_flag=1)
            self.sendCmd('P2P_traj',cmd_vals=[np.nan,r_B_O[2],0.5],cmd_flag=2)
            self.sendCmd('Activate_traj',cmd_vals=[1,1,1])
        else:
            raise Exception("Start position not approved")
        
        ## POLICY SENDING
        cmd_vals = self.userInput("Set desired (Tau,AngAcc) Policy: ",float)
        cmd_vals.append(-100) # Append extra value to match framework
        self.sendCmd('Policy',cmd_vals,cmd_flag=0)

        ## APPROVE FLIGHT
        str_input = self.userInput("Approve flight (y/n): ",str)
        if str_input == 'y':
            self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[0],self.TrajAcc_Max[0],self.TrajJerk_Max[0]],cmd_flag=0)
            self.sendCmd('Const_Vel_traj',cmd_vals=[V_B_O[2],self.TrajAcc_Max[2],self.TrajJerk_Max[2]],cmd_flag=2)
            self.sendCmd('Activate_traj',cmd_vals=[1,0,1])
        else:
            raise Exception("Flight not approved")
            

    ## ========== SYSTEM FUNCTIONS ========== 
            
    def handle_Arm_Quad(self):

        cmd_flag = self.userInput("Arm Quad On/Off (1,0): ",int)
        self.sendCmd("Arm_Quad",cmd_flag=cmd_flag)
    
    def handle_Tumble_Detect(self):

        cmd_flag = self.userInput("Tumble Detection On/Off (1,0): ",int)
        self.sendCmd('Tumble_Detect',cmd_flag=cmd_flag)

    def handle_Load_Params(self):

        print("Reset ROS Parameters\n")
        self.loadBaseParams()
        self.sendCmd("Load_Params")

    def handle_Start_Logging(self):

        self.startLogging()

    def handle_Cap_Logging(self):

        self.capLogging()

    
    ## ========== MOTOR FUNCTIONS ==========
        
    def handle_Stop(self):
        self.sendCmd("Stop")

    def handle_Thrust_CMD(self):
        vals = self.userInput("Set desired thrust values (f1,f2,f3,f4): ",float)
        self.sendCmd("Thrust_Cmd",vals[:3],vals[3])
        
    def handle_Motor_CMD(self):
        vals = self.userInput("Set desired motor values (m1,m2,m3,m4): ",float)
        self.sendCmd("Motor_Cmd",vals[:3],vals[3])

    

    # ========================
    ##    Logging Services 
    # ========================

    def createCSV(self):
        """Sends service to CF_DataConverter to create CSV log file 

        Args:
            filePath (string): Send full path and file name to write to
        """      

        ## CREATE SERVICE REQUEST MSG
        srv = Logging_CMDRequest() 
        srv.filePath = os.path.join(self.Log_Dir,self.Log_Name)
        srv.Logging_CMD = 0

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,Logging_CMD)

    def startLogging(self):
        """Start logging values to the current CSV file
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = Logging_CMDRequest()
        srv.filePath = os.path.join(self.Log_Dir,self.Log_Name)
        srv.Logging_CMD = 1

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,Logging_CMD)

    def capLogging(self):
        """Cap logging values with Flight, Rot, and Impact conditions and stop continuous logging
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = Logging_CMDRequest()
        srv.filePath = os.path.join(self.Log_Dir,self.Log_Name)
        srv.Logging_CMD = 2
        srv.error_string = self.Error_Str # String for why logging was capped
        
        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/SAR_DC/DataLogging',srv,Logging_CMD)


    # ============================
    ##   Publishers/Subscribers 
    # ============================
    def _preInit(self):

        ## COMMAND DICT
        self.cmd_dict = {
            'Ctrl_Reset':0,
            'Pos':1,
            'Vel':2,

            'Stop':5,
            'Ang_Accel':7,
            'Policy':8,
            'Plane_Pose':9,

            'P2P_traj':10,
            'Const_Vel_traj':11,
            'Activate_traj':19,

            'Tumble_Detect':20,
            'Load_Params':21,
            'Start_Logging':22,
            'Cap_Logging':23,
            'Arm_Quad':24,

            'Thrust_CMD':30,
            'Motor_CMD':31,

            'GZ_Pose_Reset':90,
            'GZ_StickyPads':91,
            'GZ_Const_Vel_Traj':92,
        }
        self.inv_cmd_dict = {value: key for key, value in self.cmd_dict.items()}

        ## RL BASED VALUES
        self.K_ep = np.nan
        self.K_run = np.nan
        self.Error_Str = "No_Debug_Data"
        self.n_rollouts = np.nan

        self.policy = np.full([3],np.nan)
        self.reward = np.nan
        self.reward_avg = np.nan
        self.reward_vals = np.full([6],np.nan)

        self.K_ep_list = []
        self.K_run_list = []

        self.mu_1_list = []
        self.mu_2_list = []

        self.sigma_1_list = []
        self.sigma_2_list = []

        self.reward_list = []
        self.Kep_list_reward_avg = []
        self.reward_avg_list = []

        ## STATE DATA VALUES
        self.t = np.nan
        self.r_B_O = np.full([3],np.nan)
        self.V_B_O = np.full([3],np.nan)
        self.eul_B_O = np.full([3],np.nan)
        self.omega_B_O = np.full([3],np.nan)

        self.r_P_B = np.full([3],np.nan)
        self.Eul_P_B = np.full([3],np.nan)
        self.V_B_P = np.full([3],np.nan)
        self.Omega_B_P = np.full([3],np.nan)
        self.Vel_mag_B_P = np.nan
        self.Vel_angle_B_P = np.nan
        self.D_perp = np.nan
        self.D_perp_CR = np.nan
        self.D_perp_pad = np.nan
        self.D_perp_pad_min = np.nan
        
        self.Theta_x = np.nan
        self.Theta_y = np.nan
        self.Tau = np.nan
        self.Tau_CR = np.nan

        ## TRIGGER DATA VALUES
        self.Trg_Flag = False
        self.t_trg = np.nan
        self.r_B_O_trg = np.full([3],np.nan)
        self.V_B_O_trg = np.full([3],np.nan)
        self.Eul_B_O_trg = np.full([3],np.nan)
        self.Omega_B_O_trg = np.full([3],np.nan)

        self.r_P_B_trg = np.full([3],np.nan)
        self.Eul_P_B_trg = np.full([3],np.nan)
        self.V_B_P_trg = np.full([3],np.nan)
        self.Omega_B_P_trg = np.full([3],np.nan)
        self.D_perp_trg = np.nan
        self.D_perp_CR_trg = np.nan

        self.Theta_x_trg = np.nan
        self.Theta_y_trg = np.nan
        self.Tau_trg = np.nan
        self.Tau_CR_trg = np.nan

        self.Vel_mag_B_O_trg = np.nan
        self.Vel_angle_B_O_trg = np.nan

        self.Vel_mag_B_P_trg = np.nan
        self.Vel_angle_B_P_trg = np.nan

        self.NN_Output_trg = np.full([4],np.nan)
        self.a_Trg_trg = np.nan
        self.a_Rot_trg = np.nan

        ## IMPACT DATA VALUES
        self.Impact_Flag = False
        
        self.Impact_Flag_OB = False
        self.t_impact_OB = np.nan
        self.r_B_O_impact_OB = np.full([3],np.nan)
        self.Eul_B_O_impact_OB = np.full([3],np.nan)
        self.V_B_P_impact_OB = np.full([3],np.nan)
        self.Omega_B_P_impact_OB = np.full([3],np.nan)
        self.Eul_P_B_impact_OB = np.full([3],np.nan)

        self.Impact_Flag_Ext = False
        self.BodyContact_Flag = False
        self.ForelegContact_Flag = False
        self.HindlegContact_Flag = False
        self.t_impact_Ext = np.nan
        self.r_B_O_impact_Ext = np.full([3],np.nan)
        self.Eul_B_O_impact_Ext = np.full([3],np.nan)
        self.V_B_P_impact_Ext = np.full([3],np.nan)
        self.Omega_B_P_impact_Ext = np.full([3],np.nan)
        self.Eul_P_B_impact_Ext = np.full([3],np.nan)
        self.Rot_Sum_impact_Ext = np.nan

        self.Force_impact_x = np.nan
        self.Force_impact_y = np.nan
        self.Force_impact_z = np.nan
        self.Impact_Magnitude = np.nan

        self.Pad_Connections = np.nan
        self.Pad1_Contact = np.nan
        self.Pad2_Contact = np.nan
        self.Pad3_Contact = np.nan
        self.Pad4_Contact = np.nan

        ## MISC DATA VALUES
        self.V_Battery = np.nan

        self.Plane_Angle_deg = np.nan
        self.Plane_Angle_rad = np.nan
        self.r_P_O = np.full([3],np.nan)

    def _clockCallback(self,msg):
        
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
        self.D_perp_pad = np.round(StateData_msg.D_perp_pad,3)
        self.D_perp_pad_min = np.round(StateData_msg.D_perp_pad_min,3)

        ## OPTICAL FLOW STATES
        self.Theta_x = np.round(StateData_msg.Optical_Flow.x,3)
        self.Theta_y = np.round(StateData_msg.Optical_Flow.y,3)
        self.Tau = np.round(StateData_msg.Optical_Flow.z,3)
        self.Tau_CR = np.round(StateData_msg.Tau_CR,3)
        
    def _SAR_TriggerDataCallback(self,TriggerData_msg):

        ## TRIGGER FLAG
        self.Trg_Flag = TriggerData_msg.Trg_Flag

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

        self.Vel_mag_B_O_trg = np.round(TriggerData_msg.Vel_mag_B_O_trg,3)
        self.Vel_angle_B_O_trg = np.round(TriggerData_msg.Vel_angle_B_O_trg,3)

        self.Vel_mag_B_P_trg = np.round(TriggerData_msg.Vel_mag_B_P_trg,3)
        self.Vel_angle_B_P_trg = np.round(TriggerData_msg.Vel_angle_B_P_trg,3)

        ## POLICY TRIGGERING CONDITIONS
        self.NN_Output_trg = np.round(TriggerData_msg.NN_Output_trg,3)
        self.a_Trg_trg = np.round(TriggerData_msg.a_Trg_trg,3)
        self.a_Rot_trg = np.round(TriggerData_msg.a_Rot_trg,3)

    def _SAR_ImpactDataCallback(self,ImpactData_msg):

        self.Impact_Flag = ImpactData_msg.Impact_Flag


        ## ONBOARD IMPACT DETECTION
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
        
        

        ## EXTERNAL IMPACT DETECTION
        self.Impact_Flag_Ext = ImpactData_msg.Impact_Flag_Ext
        self.BodyContact_Flag = ImpactData_msg.BodyContact_Flag
        self.ForelegContact_Flag = ImpactData_msg.ForelegContact_Flag
        self.HindlegContact_Flag = ImpactData_msg.HindlegContact_Flag
        
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
        self.r_P_O = [
            MiscData_msg.Plane_Pos.x,
            MiscData_msg.Plane_Pos.y,
            MiscData_msg.Plane_Pos.z,
        ]

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



    # ============================
    ##   Rotation Matrices 
    # ============================
    def R_BW(self,vec,phi):

        R_BW = np.array([
            [ np.cos(phi), 0,     np.sin(phi)],
            [     0,       1,         0      ],
            [-np.sin(phi), 0,     np.cos(phi)],
        ])

        return R_BW.dot(vec)
    
    def R_WB(self,vec,phi):

        R_WB = np.array([
            [ np.cos(phi), 0,    -np.sin(phi)],
            [     0,       1,         0      ],
            [ np.sin(phi), 0,     np.cos(phi)],
        ])

        return R_WB.dot(vec)
    
    def R_WP(self,vec,theta):

        R_WP = np.array([
            [ np.cos(theta), 0, -np.sin(theta)],
            [     0,         1,        0      ],
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
            [ np.cos(Beta1), 0, -np.sin(Beta1)],
            [     0,         1,       0       ],
            [ np.sin(Beta1), 0,  np.cos(Beta1)]
        ])

        return R_PC1.dot(vec)
    
    def R_C1P(self,vec,Beta1):

        R_C1P = np.array([
            [ np.cos(Beta1), 0, np.sin(Beta1)],
            [     0,         1,       0      ],
            [-np.sin(Beta1), 0, np.cos(Beta1)]
        ])

        return R_C1P.dot(vec)
    
    def R_C1B(self,vec,gamma_rad):

        R_C1B = np.array([
            [ np.sin(gamma_rad), 0, np.cos(gamma_rad)],
            [     0,             1,         0        ],
            [-np.cos(gamma_rad), 0, np.sin(gamma_rad)],
        ])

        return R_C1B.dot(vec)

    def R_PC2(self,vec,Beta2):

        R_PC2 = np.array([
            [ np.cos(Beta2), 0, np.sin(Beta2)],
            [     0,         1,       0      ],
            [-np.sin(Beta2), 0, np.cos(Beta2)]
        ])

        return R_PC2.dot(vec)
    
    def R_C2P(self,vec,Beta2):

        R_C2P = np.array([
            [ np.cos(Beta2), 0, np.sin(Beta2)],
            [     0,         1,       0      ],
            [-np.sin(Beta2), 0, np.cos(Beta2)],
        ])

        return R_C2P.dot(vec)

    def R_C2B(self,vec,gamma_rad):

        R_C2B = np.array([
            [-np.sin(gamma_rad), 0,  np.cos(gamma_rad)],
            [        0,          1,          0        ],
            [-np.cos(gamma_rad), 0, -np.sin(gamma_rad)],
        ])

        return R_C2B.dot(vec)

    def arctan4(self,y, x, rotation_direction=-1):
        angle_radians = np.arctan2(y, x)

        if rotation_direction == -1:
            if angle_radians < 0:
                angle_radians += 2*np.pi
        elif rotation_direction == +1:
            if angle_radians > 0:
                angle_radians = 2*np.pi - angle_radians
            else:
                angle_radians = -angle_radians

        return angle_radians



if __name__ == "__main__":

    env = SAR_Base_Interface()
    rospy.spin()


