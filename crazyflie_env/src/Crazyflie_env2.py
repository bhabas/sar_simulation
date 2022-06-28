#!/usr/bin/env python3
import numpy as np
from threading import Thread, Timer

import gym
from gym import logger,spaces

import os
import time
import sys
import subprocess
import signal
import rospy
import getpass

## ROS MESSAGES AND SERVICES
from std_srvs.srv import Empty
from crazyflie_msgs.msg import RLData,RLConvg
from crazyflie_msgs.msg import CF_StateData,CF_FlipData,CF_ImpactData,CF_MiscData
from crazyflie_msgs.srv import loggingCMD,loggingCMDRequest
from crazyflie_msgs.srv import domainRand,domainRandRequest
from crazyflie_msgs.srv import RLCmd,RLCmdRequest



from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty


class CrazyflieEnv():
    metadata = {'render.modes': ['human']}
    def __init__(self,gazeboTimeout=True,DataType='SIM'):
        super(CrazyflieEnv, self).__init__()

        rospy.init_node("crazyflie_env_node")
        os.system("roslaunch crazyflie_launch params.launch")

        self.modelName = rospy.get_param('/MODEL_NAME')
        self.mass = rospy.get_param("/CF_Mass")
        self.Ixx = rospy.get_param("/Ixx")
        self.Iyy = rospy.get_param("/Iyy")
        self.Izz = rospy.get_param("/Izz")

        ## TRAJECTORY VALUES
        self.posCF_0 = [0.0, 0.0, 0.4]        # Default hover position [m]
        self.accCF_max = [1.0, 1.0, 3.1]  # Max 5acceleration values for trajectory generation [m/s^2]

        ## GAZEBO SIMULATION INITIALIZATION
        if DataType == 'SIM': 
            self.launch_sim() 
            rospy.wait_for_service('/gazebo/pause_physics')
            self.launch_controller()

        self.launch_CF_DC()
            # print("[INITIATING] Gazebo simulation started")



        
    def step(self,action):
        obs = None
        reward = None
        done = None
        info = None
        
        return obs,reward,done,info

    def reset(self):

        ## RESET REWARD CALC VALUES
        self.d_ceil_min = 50.0  # Reset max from ceiling [m]
        self.pitch_sum = 0.0    # Reset recorded pitch amount [deg]
        self.pitch_max = 0.0    # Reset max pitch angle [deg]

        obs = None
        return obs

    def close(self):
        pass

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

        ## SEND LOGGING REQUEST VIA SERVICE
        rospy.wait_for_service('/CF_DC/Cmd_CF_DC')
        RL_Cmd_service = rospy.ServiceProxy('/CF_DC/Cmd_CF_DC', RLCmd)
        RL_Cmd_service(srv)


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
    def launch_sim(self):
        """ Launches Gazebo environment with crazyflie drone
        """        
        
        print("[STARTING] Starting Gazebo Process...")
        subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
            start_new_session=True, shell=True)

    def launch_controller(self):
        """ 
        Kill previous controller node if active and launch controller node
        """        
        
        print("[STARTING] Starting Controller Process...")
        os.system("rosnode kill /Controller_Node")
        time.sleep(0.5)
        subprocess.Popen( # Controller Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_control controller",
            close_fds=True, preexec_fn=os.setsid, shell=True)

    def launch_CF_DC(self):
        """ 
        Kill previous CF_DC node if active and launch CF_DC node
        """        
        
        print("[STARTING] Starting CF_DC Process...")
        os.system("rosnode kill /CF_DataConverter_Node")
        time.sleep(0.5)
        subprocess.Popen( # CF_DC Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_data_converter CF_DataConverter",
            close_fds=True, preexec_fn=os.setsid, shell=True)

    def close_Gazebo(self) -> bool:
        """
        Function to close gazebo if its running.
        :return: True if gazebo was closed, False otherwise.
        :rtype: bool
        """

        term_command = "rosnode kill /gazebo /gazebo_gui"
        os.system(term_command)
        time.sleep(0.5)

        term_command = "killall -9 gzserver gzclient"
        os.system(term_command)

        return True
    def sleep(self,time_s):
        """Sleep in terms of Gazebo sim seconds not real time seconds

        Args:
            time_s (_type_): _description_
        """        
        t_start = self.t
        while self.t - t_start <= time_s:
            pass # Do Nothing

    def Vel_Launch(self,pos_0,vel_d,quat_0=[0,0,0,1]): 
        """Launch crazyflie from the specified position/orientation with an imparted velocity.
        NOTE: Due to controller dynamics, the actual velocity will NOT be exactly the desired velocity

        Args:
            pos_0 (list): Launch position [m] | [x,y,z]
            vel_d (list): Launch velocity [m/s] | [Vx,Vy,Vz]
            quat_0 (list, optional): Orientation at launch. Defaults to [0,0,0,1].
        """        

        ## SET DESIRED VEL IN CONTROLLER
        self.SendCmd('Pos',cmd_flag=0)
        self.sleep(0.05)
        self.SendCmd('Vel',cmd_vals=vel_d,cmd_flag=1)

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
        self.SendCmd('StickyPads',cmd_flag=0)
        
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

        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state_srv(state_msg)
        self.SendCmd('Ctrl_Reset')
        self.sleep(0.5)

        ## RESET HOME/TUMBLE DETECTION AND STICKY

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

        self.Policy_Flip = np.round(StateData_msg.Policy_Flip,3)
        self.Policy_Action = np.round(StateData_msg.Policy_Action,3)

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
        if self.d_ceil < self.d_ceil_min:
            self.d_ceil_min = np.round(self.d_ceil,3) # Min distance achieved, used for reward calc

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


        self.Policy_Flip_tr = np.round(FlipData_msg.Policy_Flip_tr,3)
        self.Policy_Action_tr = np.round(FlipData_msg.Policy_Action_tr,3)

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


if __name__ == "__main__":

    env = CrazyflieEnv()
    input("hello")
    env.close_Gazebo()
    
    sys.exit(0)
    # while True:
    #     a = 5