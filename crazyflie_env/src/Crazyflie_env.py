#!/usr/bin/env python3
import numpy as np
from threading import Thread, Timer

import time
import sys
import os
import subprocess
import signal
import rospy
import getpass


from std_srvs.srv import Empty
from crazyflie_msgs.msg import RLData,RLCmd,RLConvg
from crazyflie_msgs.msg import CF_StateData,CF_FlipData,CF_ImpactData,CF_MiscData
from crazyflie_msgs.srv import loggingCMD,loggingCMDRequest
from crazyflie_msgs.srv import domainRand,domainRandRequest


from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState



class CrazyflieEnv:
    def __init__(self,gazeboTimeout=True,DataType='SIM'):
        print("[STARTING] CrazyflieEnv is starting...")

        rospy.init_node("crazyflie_env_node") 
        os.system("roslaunch crazyflie_launch params.launch") 

        ## GAZEBO SIMULATION INITIALIZATION
        if DataType == 'SIM': 
            self.launch_sim() 
            rospy.wait_for_message("/clock",Clock)
            self.launch_controller()
            print("[INITIATING] Gazebo simulation started")

            ## INIT GAZEBO TIMEOUT THREAD
            if gazeboTimeout==True:
                self.timeoutThread = Thread(target=self.timeoutSub)
                self.timeoutThread.start()


        self.username = getpass.getuser()
        self.loggingPath =  f"/home/{self.username}/catkin_ws/src/crazyflie_simulation/crazyflie_logging/local_logs"
        self.DataType = DataType
        self.filepath = ""
        self.trial_name = '' 
        self.error_str = ''     # Label for why rollout was terminated/completed
        self.agent_name = ''    # Learning agent used for training (PEPG,EM,etc...)
        self.runComplete_flag = False
        self.trialComplete_flag = False
        self.Logging_Flag = False
        self.repeat_run = False



        ## LOAD SIM_SETTINGS/ROS_PARAMETERS
        self.modelName = rospy.get_param('/MODEL_NAME')
        self.mass = rospy.get_param("/CF_Mass")
        self.Ixx = rospy.get_param("/Ixx")
        self.Iyy = rospy.get_param("/Iyy")
        self.Izz = rospy.get_param("/Izz")

        
        self.t_start = rospy.get_time() # [s]
        self.t_prev = 0.0 # [s]

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

        self.MS_pwm = [0,0,0,0] # Controller Motor Speeds (MS1,MS2,MS3,MS4) [PWM]
        self.MotorThrusts = [0,0,0,0] # Controller Motor Thrusts [M1,M2,M3,M4][g]
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

        

        ## INIT RL PARAMETERS
        self.h_ceiling = rospy.get_param("/CEILING_HEIGHT") # [m]

        self.n_rollouts = 6            # Rollouts per episode
        self.k_ep = 0                  # Episode number
        self.k_run = 0                 # Run number

        self.mu = [0.0,0.0]            # Gaussian mean that policies are sampled from
        self.sigma = [0.0,0.0]         # Gaussian standard deviation policies are sampled from
        self.policy = [0.0,0.0,0.0]    # Policy sampled from Gaussian distribution

        self.mu_1_list = []         # List of mu values over course of convergence
        self.mu_2_list = [] 

        self.sigma_1_list = []      # List of sigma values over course of convergence
        self.sigma_2_list = []

        self.reward_list = []       # List of reward values over course of convergence
        self.reward_avg_list = []   # List of reward averages ""

        self.reward = 0.0           # Calculated reward from run
        self.reward_avg = 0.0       # Averaged rewards over episode
        self.reward_inputs = [0.0, 0.0, 0.0] # List of inputs to reward func

        self.d_ceil_max = 50.0 # Assumed max distance from ceiling (will decrease when values measured)
        self.pitch_sum = 0.0    # Current pitch angle (roughly integrated from Wy because euler angle wrap-around issue)
        self.pitch_max = 0.0    # Max pitch angle received

        self.vel_d = [0.0,0.0,0.0] # Desired velocity for trial
     
        ## TRAJECTORY VALUES
        self.posCF_0 = [0.0, 0.0, 0.4]        # Default hover position [m]
        self.accCF_max = [1.0, 1.0, 3.1]  # Max 5acceleration values for trajectory generation [m/s^2]
      





        ## INIT ROS SUBSCRIBERS [Pub/Sampling Frequencies]
        # NOTE: Queue sizes=1 so that we are always looking at the most current data and 
        #       not data at back of a queue waiting to be processed by callbacks
        rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/FlipData",CF_FlipData,self.CF_FlipDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/ImpactData",CF_ImpactData,self.CF_ImpactDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/MiscData",CF_MiscData,self.CF_MiscDataCallback,queue_size=1)

        ## RL TOPICS 
        self.RL_Data_Publisher = rospy.Publisher('/RL/data',RLData,queue_size=10)
        self.RL_CMD_Publisher = rospy.Publisher('/RL/cmd',RLCmd,queue_size=10)
        self.RL_Convg_Publisher = rospy.Publisher('/RL/convg_data',RLConvg,queue_size=10)

        print("[COMPLETED] Environment done")

        

    # ============================
    ##   Publishers/Subscribers 
    # ============================

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
        self.d_ceil = np.round(StateData_msg.D_ceil,3)

        ## CONTROLLER ACTIONS
        self.FM = np.round([StateData_msg.FM[0],
                            StateData_msg.FM[1],
                            StateData_msg.FM[2],
                            StateData_msg.FM[3]],3)

        self.MotorThrusts = np.round([StateData_msg.MotorThrusts[0],
                                      StateData_msg.MotorThrusts[1],
                                      StateData_msg.MotorThrusts[2],
                                      StateData_msg.MotorThrusts[3]],3)

        self.MS_pwm = np.round([StateData_msg.MS_PWM[0],
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

       
        ## ======== REWARD INPUT CALCS ======== ##

        ## MIN D_CEIL CALC
        if 0.1 < self.d_ceil < self.d_ceil_max:
            self.d_ceil_max = np.round(self.d_ceil,3) # Min distance achieved, used for reward calc

        ## MAX PITCH CALC
        # Integrate omega_y over time to get full rotation estimate
        # This accounts for multiple revolutions that euler angles/quaternions can't
        self.pitch_sum = self.pitch_sum + self.omegaCF[1]*(180/np.pi)*(self.t - self.t_prev) # [deg]
        
        if self.pitch_sum < self.pitch_max:     # Recording the most negative value
            self.pitch_max = np.round(self.pitch_sum,3)

        self.t_prev = self.t # Save t value for next callback iteration

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
        if self.DataType == 'SIM':
            self.pad_connections = ImpactData_msg.Pad_Connections
            self.Pad1_Contact = ImpactData_msg.Pad1_Contact
            self.Pad2_Contact = ImpactData_msg.Pad2_Contact
            self.Pad3_Contact = ImpactData_msg.Pad3_Contact
            self.Pad4_Contact = ImpactData_msg.Pad4_Contact


    def CF_MiscDataCallback(self,MiscData_msg):        

        self.V_Battery = np.round(MiscData_msg.battery_voltage,4)
    

    def RL_Publish(self):
        """Publishes all of the RL data from
        """
        
        RL_msg = RLData() ## Initialize RLData message
        
        RL_msg.n_rollouts = self.n_rollouts

        RL_msg.k_ep = self.k_ep
        RL_msg.k_run = self.k_run
        RL_msg.error_string = self.error_str

        RL_msg.mu = self.mu
        RL_msg.sigma = self.sigma
        RL_msg.policy = self.policy

        RL_msg.reward = self.reward
        RL_msg.reward_avg = self.reward_avg
        RL_msg.reward_inputs = self.reward_inputs

        RL_msg.vel_d = self.vel_d

        RL_msg.trialComplete_flag = self.trialComplete_flag
        self.RL_Data_Publisher.publish(RL_msg) ## Publish RLData message

        ## CONVERGENCE HISTORY
        RL_convg_msg = RLConvg()

        RL_convg_msg.mu_1_list = self.mu_1_list
        RL_convg_msg.mu_2_list = self.mu_2_list

        RL_convg_msg.sigma_1_list = self.sigma_1_list
        RL_convg_msg.sigma_2_list = self.sigma_2_list

        RL_convg_msg.reward_list = self.reward_list
        RL_convg_msg.reward_avg_list = self.reward_avg_list
        self.RL_Convg_Publisher.publish(RL_convg_msg) ## Publish RLData message

    def setParams(self):
        os.system("roslaunch crazyflie_launch params.launch")
    

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

    def getTime(self):
        """Returns current known time.

        Returns:
            float: Current known time.
        """        
        
        return self.t
        
    def reset_reward_terms(self):
        """Reset values used in reward calculation.
        """        

        ## RESET REWARD CALC VALUES
        self.d_ceil_max = 50.0  # Reset max from ceiling [m]
        self.pitch_sum = 0.0    # Reset recorded pitch amount [deg]
        self.pitch_max = 0.0    # Reset max pitch angle [deg]

    def step(self,action,cmd_vals=[0,0,0],cmd_flag=1):
        """Sends commands to Crazyflie controller via rostopic

        Args:
            action (string): The desired command
            cmd_vals (list, optional): Command values typically in [x,y,z] notation. Defaults to [0,0,0].
            cmd_flag (float, optional): Used as either a on/off flag for command or an extra float value if needed. Defaults to 1.
        """        

        cmd_msg = RLCmd()   # Create message object
        cmd_dict = {
            'home':0,       # Resets controller values to defaults
            'pos':1,        # Set desired position values (Also on/off for position control)
            'vel':2,        # Set desired vel values (Also on/off for vel control)
            'acc':3,        # --- Unused ---
            'tumble':4,     # Turns tumble detection on/off (Uneeded?)
            'stop':5,       # Cutoff motor values
            'params':6,     # Reloads ROS params and updates params in controller
            'moment':7,     # Execute the desired moment in terms of [Mx,My,Mz] in [N*mm]
            'policy':8,     # Activate policy triggering and policy values [tau_thr,My,G2]
            'sticky':11,    # Turns on/off sticky legs via cmd_flag

            'thrusts':10,   # Controls individual motor thrusts [M1,M2,M3,M4]
            'M_PWM':12,     # Control individual motor pwm values (bypasses thrust values)

            'vel_traj':9,   # Execute constant velocity trajectory cmd_val=[s_0,v_0,a_0] | cmd_flag=[x:0,y:1,z:2]
            'P2P_traj':13   # Execute point-to-point trajectory cmd_vals=[s_0,s_f,a_0] | cmd_flag=[x:0,y:1,z:2]
        }
        

        ## INSERT VALUES TO ROS MSG
        cmd_msg.cmd_type = cmd_dict[action]
        cmd_msg.cmd_vals.x = cmd_vals[0]
        cmd_msg.cmd_vals.y = cmd_vals[1]
        cmd_msg.cmd_vals.z = cmd_vals[2]
        cmd_msg.cmd_flag = cmd_flag
        
        ## PUBLISH MESSAGE
        self.RL_CMD_Publisher.publish(cmd_msg) 
        time.sleep(0.02) # Give time for controller to process message
        

    def VelTraj_StartPos(self,x_impact,V_d,accel_d=None,d_vel=0.6):
        """Returns the required start position (x_0,z_0) to intercept the ceiling 
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

        ## DEFAULT TO CLASS VALUES
        if accel_d == None:
            accel_d = self.accCF_max
    
        a_x = accel_d[0]
        a_z = accel_d[2]

        ## CALC OFFSET POSITIONS
        Vx = V_d[0]
        Vz = V_d[2]

        t_x = Vx/a_x    # Time required to reach Vx
        t_z = Vz/a_z    # Time required to reach Vz

        z_vz = 1/2*a_z*t_z**2                   # Height Vz reached
        z_0 = (self.h_ceiling - d_vel) - z_vz    # Offset to move z_vz to d_vz
        
        x_vz = Vx*(t_x+t_z) - Vx**2/(2*a_x)     # X-position Vz reached
        x_0 = x_impact - x_vz - d_vel*Vx/Vz      # Account for shift up and shift left

        return x_0,z_0

    # ========================
    ##    Logging Services 
    # ========================

    def createCSV(self,filePath):
        """Sends service to CF_DataConverter to create CSV file to write logs to

        Args:
            filePath (string): Send full path and file name to write to
        """      

        ## CREATE SERVICE REQUEST MSG
        srv = loggingCMDRequest() 

        ## CREATE CSV COMMANDS
        srv.createCSV = True
        srv.filePath = filePath

        ## MAKE SURE LOGGING IS TURNED OFF
        srv.Logging_Flag = False
        self.Logging_Flag = srv.Logging_Flag
        
        ## SEND LOGGING REQUEST VIA SERVICE
        rospy.wait_for_service('/CF_DC/DataLogging')
        logging_service = rospy.ServiceProxy('/CF_DC/DataLogging', loggingCMD)
        logging_service(srv)

    def startLogging(self):
        """Start logging values to the current CSV file
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = loggingCMDRequest()

        ## CREATE CSV COMMANDS
        srv.Logging_Flag = True
        self.Logging_Flag = srv.Logging_Flag

        ## SEND LOGGING REQUEST VIA SERVICE
        rospy.wait_for_service('/CF_DC/DataLogging')
        logging_service = rospy.ServiceProxy('/CF_DC/DataLogging', loggingCMD)
        logging_service(srv)

    def capLogging(self):
        """Cap logging values with IC,Flip, and Impact conditions and stop logging
        """        

        ## CREATE SERVICE REQUEST MSG
        srv = loggingCMDRequest()

        ## STOP LOGGING
        srv.Logging_Flag = False
        self.Logging_Flag = srv.Logging_Flag

        srv.capLogging = True
        srv.error_string = self.error_str # String for why logging was capped
        
        ## SEND LOGGING REQUEST VIA SERVICE
        rospy.wait_for_service('/CF_DC/DataLogging')
        logging_service = rospy.ServiceProxy('/CF_DC/DataLogging', loggingCMD)
        logging_service(srv)
    

    # ============================
    ##       GAZEBO OPERATION
    # ============================
    

    def relaunch_sim(self):
        """
        Relaunches Gazebo and resets model position but doesn't touch controller node
        """        
        self.close_sim()
        time.sleep(5.0)
        self.launch_sim()
        self.reset_pos()

    def close_sim(self):
        """ 
        Terminates gazebo process
        """        
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)
        
    def close_controller(self):
        """ Terminates controller process
        """        
        os.killpg(self.controller_p.pid, signal.SIGTERM)

    def close_dashboard(self):
        """ Terminates dashboard process
        """        
        os.killpg(self.dashboard_p.pid, signal.SIGTERM)


    def close_proc(self):
        """Cleanly shut downs gazebo/controller node when shutdown command recieved (CTRL+c)
        """        
        os.system("killall gzserver gzclient")
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)
        os.killpg(self.controller_p.pid, signal.SIGTERM)
        sys.exit(0)
    
    def launch_sim(self):
        """ Launches Gazebo environment with crazyflie drone
        """        
        
        print("[STARTING] Starting Gazebo Process...")
        self.gazebo_p = subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
            start_new_session=True, shell=True)
  
    def pause_sim(self,pause_flag):
        """Pauses simulation

        Args:
            pause_flag (bool): On/Off pause of simulation
        """        

        if pause_flag:
            rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        else:
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)


    def launch_dashboard(self):
        """ Launch dashboard subprocess
        """        
        print("[STARTING] Starting Dashboard...")
        self.dashboard_p = subprocess.Popen(
            "gnome-terminal -- roslaunch crazyflie_launch dashboard.launch",
            close_fds=True, preexec_fn=os.setsid, shell=True)
    
    def launch_controller(self):
        """
        Kill previous controller node if active and launch controller node
        """        
        print("[STARTING] Starting Controller Process...")
        os.system("rosnode kill /controller_node")
        self.controller_p = subprocess.Popen( # Controller Process
            "roslaunch crazyflie_launch controller.launch",
            close_fds=True, preexec_fn=os.setsid, shell=True)


    def Vel_Launch(self,pos_0,vel_d,quat_0=[0,0,0,1]): 
        """Launch crazyflie from the specified position/orientation with an imparted velocity.
        NOTE: Due to controller dynamics, the actual velocity will NOT be exactly the desired velocity

        Args:
            pos_0 (list): Launch position [m] | [x,y,z]
            vel_d (list): Launch velocity [m/s] | [Vx,Vy,Vz]
            quat_0 (list, optional): Orientation at launch. Defaults to [0,0,0,1].
        """        

        ## SET DESIRED VEL IN CONTROLLER
        self.step('pos',cmd_flag=0)
        self.step('vel',cmd_vals=vel_d,cmd_flag=1)

        ## CREATE SERVICE MESSAGE
        state_msg = ModelState()
        state_msg.model_name = self.modelName

        ## INPUT POSITION AND ORIENTATION
        state_msg.pose.position.x = pos_0[0]
        state_msg.pose.position.y = pos_0[1]
        state_msg.pose.position.z = pos_0[2]

        state_msg.pose.orientation.x = quat_0[0]
        state_msg.pose.orientation.y = quat_0[1]
        state_msg.pose.orientation.z = quat_0[2]
        state_msg.pose.orientation.w = quat_0[3]

        ## INPUT LINEAR AND ANGULAR VELOCITY
        state_msg.twist.linear.x = vel_d[0]
        state_msg.twist.linear.y = vel_d[1]
        state_msg.twist.linear.z = vel_d[2]

        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0

        ## PUBLISH MODEL STATE SERVICE REQUEST
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state_service(state_msg)
                

    def reset_pos(self,z_0=0.379): # Disable sticky then places spawn_model at origin
        """Reset pose/twist of simulated drone back to home position. 
        As well as turning off stickyfeet

        Args:
            z_0 (float, optional): Starting height of crazyflie. Defaults to 0.379.
        """        
        ## DISABLE STICKY LEGS (ALSO BREAKS CURRENT CONNECTION JOINTS)
        self.step('sticky',cmd_flag=0)
        time.sleep(0.05)
        
        ## RESET POSITION AND VELOCITY
        state_msg = ModelState()
        state_msg.model_name = self.modelName
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0.0
        state_msg.pose.position.z = z_0

        state_msg.pose.orientation.w = 1.0
        state_msg.pose.orientation.x = 0.0
        state_msg.pose.orientation.y = 0.0
        state_msg.pose.orientation.z = 0.0
        
        state_msg.twist.linear.x = 0.0
        state_msg.twist.linear.y = 0.0
        state_msg.twist.linear.z = 0.0

        state_msg.twist.angular.x = 0.0
        state_msg.twist.angular.y = 0.0
        state_msg.twist.angular.z = 0.0

        rospy.wait_for_service('/gazebo/set_model_state',timeout=5)
        set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state_srv(state_msg)

        ## RESET HOME/TUMBLE DETECTION AND STICKY
        # self.step('tumble',cmd_flag=1) # Tumble Detection On
        self.step('home')

    def updateInertia(self):

        ## CREATE SERVICE REQUEST MSG
        srv = domainRandRequest() 
        srv.mass = self.mass
        srv.Inertia.x = self.Ixx
        srv.Inertia.y = self.Iyy
        srv.Inertia.z = self.Izz

        ## SEND LOGGING REQUEST VIA SERVICE
        try:
            rospy.wait_for_service('/CF_Internal/DomainRand',timeout=1.0)
            domainRand_service = rospy.ServiceProxy('/CF_Internal/DomainRand', domainRand)
            domainRand_service(srv)
        except rospy.exceptions.ROSException:
            
            pass
       
   

    # ============================
    ##      Timeout Functions 
    # ============================

    # Subscriber thread listens to /clock for any message
    def timeoutSub(self):
        ## START INITIAL TIMERS
        self.timer_unpause = Timer(5,self.timeout_unpause)
        self.timer_relaunch = Timer(10,self.timeout_relaunch)
        ## START ROS CREATED THREAD FOR SUBSCRIBER
        rospy.Subscriber("/clock",Clock,self.timeoutCallback)
        ## END FUNCTION, THIS MIGHT NOT NEED TO BE THREADED?

    
    # If message is received reset the threading.Timer threads
    def timeoutCallback(self,msg):
        ## RESET TIMER THAT ATTEMPTS TO UNPAUSE SIM
        self.timer_unpause.cancel()
        self.timer_unpause = Timer(4.0,self.timeout_unpause)
        self.timer_unpause.start()

        ## RESET TIMER THAT RELAUNCHES SIM
        self.timer_relaunch.cancel()
        self.timer_relaunch = Timer(7.0,self.timeout_relaunch)
        self.timer_relaunch.start()
    

    def timeout_unpause(self):
        print("[UNPAUSING] No Gazebo communication in 4 seconds")
        os.system("rosservice call gazebo/unpause_physics")

    def timeout_relaunch(self):
        print("[RELAUNCHING] No Gazebo communication in 7 seconds")
        self.close_sim()
        time.sleep(1)
        self.launch_sim()
        time.sleep(1)



    




if __name__ == "__main__":
    env = CrazyflieEnv()
    rospy.on_shutdown(env.close_proc)

    rospy.spin()