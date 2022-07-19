#!/usr/bin/env python3
import numpy as np
import os
import rospy
import getpass

## ROS MESSAGES AND SERVICES
from crazyflie_msgs.msg import CF_StateData,CF_FlipData,CF_ImpactData,CF_MiscData
from crazyflie_msgs.srv import loggingCMD,loggingCMDRequest
from crazyflie_msgs.srv import RLCmd,RLCmdRequest

from rosgraph_msgs.msg import Clock

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
ENDC = '\033[m'

class CrazyflieEnv_Base():
    metadata = {'render.modes': ['human']}
    def __init__(self):
        os.system("roslaunch crazyflie_launch params.launch")

        self.modelName = rospy.get_param('/MODEL_NAME')
        self.h_ceiling = rospy.get_param("/CEILING_HEIGHT") # [m]
        self.env_name = "CF_Base"
     
        ## TRAJECTORY VALUES
        self.posCF_0 = [0.0, 0.0, 0.4]      # Default hover position [m]
        self.accCF_max = [1.0, 1.0, 3.1]    # Max acceleration values for trajectory generation [m/s^2]

        self.username = getpass.getuser()
        self.logDir =  f"/home/{self.username}/catkin_ws/src/crazyflie_simulation/crazyflie_logging/local_logs"
        self.logName = "TestLog.csv"
        self.error_str = ""


        self.preInit_Values()

        ## INIT ROS SUBSCRIBERS [Pub/Sampling Frequencies]
        # NOTE: Queue sizes=1 so that we are always looking at the most current data and 
        #       not data at back of a queue waiting to be processed by callbacks
        rospy.Subscriber("/clock",Clock,self.clockCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/FlipData",CF_FlipData,self.CF_FlipDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/ImpactData",CF_ImpactData,self.CF_ImpactDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/MiscData",CF_MiscData,self.CF_MiscDataCallback,queue_size=1)


    def getTime(self):
        """Returns current known time.

        Returns:
            float: Current known time.
        """        
        
        return self.t

    def SendCmd(self,action,cmd_vals=[0,0,0],cmd_flag=1):
        """Sends commands to Crazyflie controller via rostopic

        Args:
            action (string): The desired command
            cmd_vals (list, optional): Command values typically in [x,y,z] notation. Defaults to [0,0,0].
            cmd_flag (float, optional): Used as either a on/off flag for command or an extra float value if needed. Defaults to 1.
        """        

        cmd_dict = {
            'Ctrl_Reset':0,
            'Pos':1,
            'Vel':2,
            'Stop':5,
            'Moment':7,
            'Policy':8,

            'P2P_traj':10,
            'Vel_traj':11,
            'Impact_traj':12,

            'Tumble':20,
            'Load_Params':21,
            'Cap_Logging':22,

            'GZ_traj':90,
            'GZ_reset':91,
            'StickyPads':92,
        }

        ## CREATE SERVICE REQUEST MSG
        srv = RLCmdRequest() 
        
        srv.cmd_type = cmd_dict[action]
        srv.cmd_vals.x = cmd_vals[0]
        srv.cmd_vals.y = cmd_vals[1]
        srv.cmd_vals.z = cmd_vals[2]
        srv.cmd_flag = cmd_flag

        self.callService('/CF_DC/Cmd_CF_DC',srv,RLCmd)        

    def callService(self,addr,srv,srv_type,retries=5):
  
        for retry in range(retries):
            try:
                service = rospy.ServiceProxy(addr, srv_type)
                service(srv)
                return True

            except rospy.ServiceException as e:
                print(f"[WARNING] {addr} service call failed (callService)")
                print(f"[WARNING] {e}")

        return False

                
    def preInit_Values(self):

        self.Ixx = rospy.get_param("/Ixx")
        self.Iyy = rospy.get_param("/Iyy")
        self.Izz = rospy.get_param("/Izz")
        self.mass = rospy.get_param("/CF_Mass")
        
        ## RAW VICON VALUES
        self.posViconRaw = [0,0,0]
        self.quatViconRaw = [0,0,0,1]

        ## FILTERED VICON STATES
        self.posVicon = [0,0,0]
        self.velVicon = [0,0,0]

        self.quatVicon = [0,0,0,1]
        self.eulVicon = [0,0,0]
        self.omegaVicon = [0,0,0]


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
        self.d_ceil = 0.0 

        self.MS_pwm = [0,0,0,0]         # Controller Motor Speeds (MS1,MS2,MS3,MS4) [PWM]
        self.MotorThrusts = [0,0,0,0]   # Controller Motor Thrusts [M1,M2,M3,M4][g]
        self.FM = [0,0,0,0]             # Controller Force/Moments (F_thrust,Mx,My,Mz) [N,N*mm]
        
        self.Policy_Flip = 0.0
        self.Policy_Action = 0.0
        
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
        self.OFx_tr = 0.0           # [rad/s]
        self.OFy_tr = 0.0           # [rad/s]
        self.d_ceil_tr = 0.0        # [m]

        self.FM_tr = [0,0,0,0]      # [N,N*mm]

        self.Policy_Flip_tr = 0.0
        self.Policy_Action_tr = 0.0     # [N*mm]

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


    # ========================
    ##    Logging Services 
    # ========================

    def createCSV(self,logName):
        """Sends service to CF_DataConverter to create CSV file to write logs to

        Args:
            filePath (string): Send full path and file name to write to
        """      

        ## CREATE SERVICE REQUEST MSG
        srv = loggingCMDRequest() 
        srv.filePath = f"{self.logDir}/{logName}"
        srv.Logging_CMD = 0

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/CF_DC/DataLogging',srv,loggingCMD)

    def startLogging(self,logName):
        """Start logging values to the current CSV file
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = loggingCMDRequest()
        srv.filePath = f"{self.logDir}/{logName}"
        srv.Logging_CMD = 1

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/CF_DC/DataLogging',srv,loggingCMD)

    def capLogging(self,logName):
        """Cap logging values with IC,Flip, and Impact conditions and stop logging
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = loggingCMDRequest()
        srv.filePath = f"{self.logDir}/{logName}"
        srv.Logging_CMD = 2
        srv.error_string = self.error_str # String for why logging was capped
        
        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService('/CF_DC/DataLogging',srv,loggingCMD)


    # ============================
    ##   Publishers/Subscribers 
    # ============================
    def clockCallback(self,msg):
        self.t = msg.clock.to_sec()

    def CF_StateDataCallback(self,StateData_msg):

        # self.t = msg.clock.to_sec()

        self.posCF = np.round([ StateData_msg.Pose.position.x,
                                StateData_msg.Pose.position.y,
                                StateData_msg.Pose.position.z],3)

        self.eulCF = np.round([ StateData_msg.Eul.x,
                                StateData_msg.Eul.y,
                                StateData_msg.Eul.z],3)

        ## CF_TWIST
        self.velCF = np.round([ StateData_msg.Twist.linear.x,
                                StateData_msg.Twist.linear.y,
                                StateData_msg.Twist.linear.z],3)

        ## CF_VISUAL STATES
        self.Tau = np.round(StateData_msg.Tau,3)
        self.OFx = np.round(StateData_msg.OFx,3)
        self.OFy = np.round(StateData_msg.OFy,3)
        self.d_ceil = np.round(StateData_msg.D_ceil,3)
       
        self.t_prev = self.t # Save t value for next callback iteration

    def CF_FlipDataCallback(self,FlipData_msg):

        ## FLIP FLAGS
        self.flip_flag = FlipData_msg.flip_flag

    def CF_ImpactDataCallback(self,ImpactData_msg):

        ## IMPACT FLAGS
        self.impact_flag = ImpactData_msg.impact_flag
        self.BodyContact_flag = ImpactData_msg.BodyContact_flag

        self.eulCF_impact = np.round([ImpactData_msg.Eul_impact.x,
                                      ImpactData_msg.Eul_impact.y,
                                      ImpactData_msg.Eul_impact.z],3)

        ## CF_TWIST (IMPACT)
        self.velCF_impact = np.round([ImpactData_msg.Twist_impact.linear.x,
                                      ImpactData_msg.Twist_impact.linear.y,
                                      ImpactData_msg.Twist_impact.linear.z],3)

        ## STICKY PAD CONNECTIONS
        self.pad_connections = ImpactData_msg.Pad_Connections


    def CF_MiscDataCallback(self,MiscData_msg):        

        self.V_Battery = np.round(MiscData_msg.battery_voltage,4)

    def modelInitials(self):
        """Returns initials for the model

        Returns:
            string: Model name initials
        """        
        str = self.modelName
        charA = str[self.modelName.find("_")+1] # [W]ide
        charB = str[self.modelName.find("-")+1] # [L]ong

        return charA+charB  # [WL]

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

    env = CrazyflieEnv_Base()


