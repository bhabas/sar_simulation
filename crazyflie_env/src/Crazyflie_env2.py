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
    def __init__(self,gazeboTimeout=False,DataType='SIM'):
        super(CrazyflieEnv, self).__init__()

        rospy.init_node("crazyflie_env_node")
        os.system("roslaunch crazyflie_launch params.launch")

        self.modelName = rospy.get_param('/MODEL_NAME')
        self.h_ceiling = rospy.get_param("/CEILING_HEIGHT") # [m]

        ## TRAJECTORY VALUES
        self.posCF_0 = [0.0, 0.0, 0.4]        # Default hover position [m]
        self.accCF_max = [1.0, 1.0, 3.1]  # Max 5acceleration values for trajectory generation [m/s^2]

        self.username = getpass.getuser()
        self.loggingPath =  f"/home/{self.username}/catkin_ws/src/crazyflie_simulation/crazyflie_logging/local_logs"
        self.DataType = DataType
        self.filepath = ""

        self.d_min = 50.0
        self.t = 0.0
        self.restart_flag = False

        ## GAZEBO SIMULATION INITIALIZATION
        if self.DataType == 'SIM': 
            self.launch_Gazebo() 
            rospy.wait_for_service('/gazebo/pause_physics')
            self.launch_Node_Controller()
            time.sleep(1.0)
            self.gazebo_unpause_physics()
            print("[INITIATING] Gazebo simulation started")

            ## INIT GAZEBO TIMEOUT THREAD
            if gazeboTimeout==True:
                self.timeoutThread = Thread(target=self.timeoutSub)
                self.timeoutThread.start()
            

        self.launch_Node_CF_DC()
            # print("[INITIATING] Gazebo simulation started")

        ## INIT ROS SUBSCRIBERS [Pub/Sampling Frequencies]
        # NOTE: Queue sizes=1 so that we are always looking at the most current data and 
        #       not data at back of a queue waiting to be processed by callbacks
        rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/FlipData",CF_FlipData,self.CF_FlipDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/ImpactData",CF_ImpactData,self.CF_ImpactDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/MiscData",CF_MiscData,self.CF_MiscDataCallback,queue_size=1)


    def ParamOptim_Flight(self,Tau,My,vel,phi):

        done = False

        ## RESET/UPDATE RUN CONDITIONS
        start_time_rollout = self.getTime()
        start_time_pitch = np.nan
        start_time_impact = np.nan

        ## RESET LOGGING CONDITIONS 
        onceFlag_flip = False    # Ensures flip data recorded only once
        onceFlag_impact = False   # Ensures impact data recorded only once 


        self.SendCmd('StickyPads',cmd_flag=1)

        vz = vel*np.sin(np.deg2rad(phi))
        vx = vel*np.cos(np.deg2rad(phi))

        tau_0 = 0.6
        z_0 = self.h_ceiling - tau_0*vz

        self.Vel_Launch([0,0,z_0],[vx,0,vz])
        self.sleep(0.1)
        self.SendCmd("Policy",cmd_vals=[Tau,My,0.0],cmd_flag=1)


        

        while not done: 

            t_now = self.getTime()

            ## START FLIP AND IMPACT TERMINATION TIMERS
            if (self.flip_flag == True and onceFlag_flip == False):
                start_time_pitch = t_now # Starts countdown for when to reset run
                onceFlag_flip = True # Turns on to make sure this only runs once per rollout

            if ((self.impact_flag or self.BodyContact_flag) and onceFlag_impact == False):
                start_time_impact = t_now
                onceFlag_impact = True

            # ============================
            ##    Termination Criteria 
            # ============================

            ## PITCH TIMEOUT  
            if (t_now-start_time_pitch) > 2.25:
                self.error_str = "Rollout Completed: Pitch Timeout"
                print(self.error_str)

                done = True

            ## IMPACT TIMEOUT
            elif (t_now-start_time_impact) > 0.5:
                self.error_str = "Rollout Completed: Impact Timeout"
                print(self.error_str)

                done = True


            ## ROLLOUT TIMEOUT
            elif (t_now - start_time_rollout) > (5.0):
                self.error_str = "Rollout Completed: Time Exceeded"
                print(self.error_str)

                done = True

            ## FREE FALL TERMINATION
            elif self.velCF[2] <= -0.5 and self.posCF[2] <= 1.5: 
                self.error_str = "Rollout Completed: Falling Drone"
                print(self.error_str)

                done = True


            if any(np.isnan(self.velCF)): 
            
                self.close_Gazebo()
                time.sleep(0.5)
                self.launch_Gazebo() 
                rospy.wait_for_service('/gazebo/pause_physics')
                time.sleep(0.5)
                self.gazebo_unpause_physics()

                done = True


        reward = self.CalcReward()

        return None,reward,done,None

        
    def step(self,action):
        obs = None
        reward = None
        done = None
        info = None
        
        return obs,reward,done,info

    def reset(self):

        self.reset_pos()
        self.sleep(0.25)

        ## RESET REWARD CALC VALUES
        self.d_min = 50.0  # Reset max from ceiling [m]

        obs = None
        return obs

    def close(self):
        self.close_Gazebo()

    def CalcReward(self):

        ## PAD CONTACT REWARD
        if self.pad_connections >= 3: 
            if self.BodyContact_flag == False:
                R1 = 0.7
            else:
                R1 = 0.4
            
        elif self.pad_connections == 2: 
            if self.BodyContact_flag == False:
                R1 = 0.3
            else:
                R1 = 0.2
        
        else:
            R1 = 0.0

        ## IMPACT ANGLE REWARD
        if -180 <= self.eulCF_impact[1] <= -90:
            R2 = 1.0
        elif -90 < self.eulCF_impact[1] <= 0:
            R2 = -1/90*self.eulCF_impact[1]
        elif 0 < self.eulCF_impact[1] <= 90:
            R2 = 1/90*self.eulCF_impact[1]
        elif 90 < self.eulCF_impact[1] <= 180:
            R2 = 1.0
        else:
            R2 = 0

        R2 *= 0.2

        ## DISTANCE REWARD (Gaussian Function)
        A = 0.1
        mu = 0
        sig = 1
        R3 = A*np.exp(-1/2*np.power((self.d_min-mu)/sig,2))

        return R1 + R2 + R3

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
        self.sleep(0.01)
        self.SendCmd('Ctrl_Reset')

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
    ##      GAZEBO TECHNICAL
    # ============================
    def launch_Gazebo(self):
        """ Launches Gazebo environment with crazyflie drone
        """        
        
        print("[STARTING] Starting Gazebo Process...")
        subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
            start_new_session=True, shell=True)

    def launch_Node_Controller(self):
        """ 
        Kill previous controller node if active and launch controller node
        """        
        
        print("[STARTING] Starting Controller Process...")
        os.system("rosnode kill /Controller_Node")
        time.sleep(0.5)
        subprocess.Popen( # Controller Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_control controller",
            close_fds=True, preexec_fn=os.setsid, shell=True)

    def launch_Node_CF_DC(self):
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

    def gazebo_pause_physics(self,retries=5) -> bool:
        """
        Function to pause the physics in the simulation.
        :param retries: The number of times to retry the service call.
        :type retries: int
        :return: True if the command was sent and False otherwise.
        :rtype: bool
        """

        rospy.wait_for_service("/gazebo/pause_physics")
        client_srv = rospy.ServiceProxy("/gazebo/pause_physics", Empty)

        for retry in range(retries):
            try:
                client_srv()
                return True
            except rospy.ServiceException as e:
                print ("/gazebo/pause_physics service call failed")
            
        return False

    def gazebo_unpause_physics(self,retries=5) -> bool:
        """
        Function to unpause the physics in the simulation.
        :param retries: The number of times to retry the service call.
        :type retries: int
        :return: True if the command was sent and False otherwise.
        :rtype: bool
        """

        rospy.wait_for_service("/gazebo/unpause_physics")
        client_srv = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        
        for retry in range(retries):
            try:
                client_srv()
                return True
            except rospy.ServiceException as e:
                print ("/gazebo/pause_physics service call failed")
            
        return False

    # ============================
    ##   Publishers/Subscribers 
    # ============================

    def CF_StateDataCallback(self,StateData_msg):

        self.t = np.round(StateData_msg.header.stamp.to_sec(),4)

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
       
        ## ======== REWARD INPUT CALCS ======== ##

        ## MIN D_CEIL CALC
        if self.d_ceil < self.d_min:
            self.d_min = np.round(self.d_ceil,3) # Min distance achieved, used for reward calc

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
        if self.DataType == 'SIM':
            self.pad_connections = ImpactData_msg.Pad_Connections
            self.Pad1_Contact = ImpactData_msg.Pad1_Contact
            self.Pad2_Contact = ImpactData_msg.Pad2_Contact
            self.Pad3_Contact = ImpactData_msg.Pad3_Contact
            self.Pad4_Contact = ImpactData_msg.Pad4_Contact

    def CF_MiscDataCallback(self,MiscData_msg):        

        self.V_Battery = np.round(MiscData_msg.battery_voltage,4)

    # ============================
    ##      Timeout Functions 
    # ============================

    # Subscriber thread listens to /clock for any message
    def timeoutSub(self):
        ## START INITIAL TIMERS
        self.timer_unpause = Timer(4,self.timeout_unpause)
        self.timer_relaunch = Timer(7,self.timeout_relaunch)
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
        print("[UNPAUSING] No Gazebo communication in 5 seconds")
        os.system("rosservice call gazebo/unpause_physics")

    def timeout_relaunch(self):
        print("[RELAUNCHING] No Gazebo communication in 10 seconds")
        self.close_Gazebo()
        time.sleep(1)
        self.launch_Gazebo()
        time.sleep(1)

if __name__ == "__main__":

    env = CrazyflieEnv(gazeboTimeout=False)
    for ii in range(1000):
        tau = np.random.uniform(low=0.15,high=0.27)

        env.reset()
        obs,reward,done,info = env.ParamOptim_Flight(tau,7,2.5,60)
        print(ii)


    env.close()