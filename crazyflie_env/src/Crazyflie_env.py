#!/usr/bin/env python3
import numpy as np
import warnings

import gym
from gym import logger,spaces

import os
import time
import sys
import subprocess
import rospy
import getpass

## ROS MESSAGES AND SERVICES
from std_srvs.srv import Empty
from crazyflie_msgs.msg import CtrlData
from crazyflie_msgs.msg import CF_StateData,CF_FlipData,CF_ImpactData,CF_MiscData
from crazyflie_msgs.srv import loggingCMD,loggingCMDRequest
from crazyflie_msgs.srv import domainRand,domainRandRequest
from crazyflie_msgs.srv import RLCmd,RLCmdRequest



from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
ENDC = '\033[m'

class CrazyflieEnv():
    metadata = {'render.modes': ['human']}
    def __init__(self,gazeboTimeout=False,DataType='SIM'):
        super(CrazyflieEnv, self).__init__()        
        os.system("roslaunch crazyflie_launch params.launch")

        self.modelName = rospy.get_param('/MODEL_NAME')
        self.h_ceiling = rospy.get_param("/CEILING_HEIGHT") # [m]

        ## TRAJECTORY VALUES
        self.posCF_0 = [0.0, 0.0, 0.4]        # Default hover position [m]
        self.accCF_max = [1.0, 1.0, 3.1]  # Max 5acceleration values for trajectory generation [m/s^2]

        self.username = getpass.getuser()
        self.logDir =  f"/home/{self.username}/catkin_ws/src/crazyflie_simulation/crazyflie_logging/local_logs"
        self.logName = "TestLog.csv"

        self.DataType = DataType
        self.error_str = ""

        self.d_min = 50.0
        self.done = False

        high = np.array(
            [
                np.finfo(np.float32).max,
                np.finfo(np.float32).max,
                np.finfo(np.float32).max,
            ],
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-5]), high=np.array([5]), shape=(1,), dtype=np.float32)

        ## GAZEBO SIMULATION INITIALIZATION
        if self.DataType == 'SIM': 
            
            self.gazeboTimeout = gazeboTimeout
            rospy.init_node("crazyflie_env_node")

            ## LAUNCH GAZEBO
            self.launch_Gazebo() 
            rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
            
            ## LAUNCH CONTROLLER
            self.launch_Node_Controller()
            rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=5)

            ## LAUNCH CF_DC
            self.launch_Node_CF_DC()
            rospy.wait_for_service("/CF_DC/Cmd_CF_DC",timeout=5)

            print("[INITIATING] Gazebo simulation started")
            

        self.preInit_Values()

        ## INIT ROS SUBSCRIBERS [Pub/Sampling Frequencies]
        # NOTE: Queue sizes=1 so that we are always looking at the most current data and 
        #       not data at back of a queue waiting to be processed by callbacks
        rospy.Subscriber("/clock",Clock,self.clockCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/StateData",CF_StateData,self.CF_StateDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/FlipData",CF_FlipData,self.CF_FlipDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/ImpactData",CF_ImpactData,self.CF_ImpactDataCallback,queue_size=1)
        rospy.Subscriber("/CF_DC/MiscData",CF_MiscData,self.CF_MiscDataCallback,queue_size=1)

    

    def ParamOptim_Flight(self,Tau,My,vel,phi):

        ## RESET/UPDATE RUN CONDITIONS
        start_time_rollout = self.getTime()
        start_time_pitch = np.nan
        start_time_impact = np.nan

        start_time_ep = time.time()

        ## RESET LOGGING CONDITIONS 
        onceFlag_flip = False    # Ensures flip data recorded only once
        onceFlag_impact = False   # Ensures impact data recorded only once 


        self.SendCmd('StickyPads',cmd_flag=1)

        vz = vel*np.sin(np.deg2rad(phi))
        vx = vel*np.cos(np.deg2rad(phi))

        tau_0 = 0.6
        z_0 = self.h_ceiling - tau_0*vz

        self.Vel_Launch([0,0,z_0],[vx,0,vz])
        self.SendCmd("Policy",cmd_vals=[Tau,My,0.0],cmd_flag=1)


        

        while not self.done: 

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
                self.done = True
                # print(self.error_str)


            ## IMPACT TIMEOUT
            elif (t_now-start_time_impact) > 0.5:
                self.error_str = "Rollout Completed: Impact Timeout"
                self.done = True
                # print(self.error_str)



            ## ROLLOUT TIMEOUT
            elif (t_now - start_time_rollout) > 5.0:
                self.error_str = "Rollout Completed: Time Exceeded"
                self.done = True
                # print(self.error_str)

            ## FREE FALL TERMINATION
            elif self.velCF[2] <= -0.5 and self.posCF[2] <= 1.5: 
                self.error_str = "Rollout Completed: Falling Drone"
                self.done = True
                # print(self.error_str)



            if (time.time() - start_time_ep) > 15.0 and self.gazeboTimeout == True:
                print('\033[93m' + "[WARNING] Real Time Exceeded" + '\x1b[0m')
                self.Restart()
                self.done = True


            if any(np.isnan(self.velCF)): 
                self.error_str = "Rollout Completed: NaN in State Vector"

                print('\033[93m' + "[WARNING] NaN in State Vector" + '\x1b[0m')
                self.Restart([True,False,False])
                print('\033[93m' + "[WARNING] Resuming Flight" + '\x1b[0m')
                self.done = True
                # print(self.error_str)





        reward = self.CalcReward()

        return None,reward,self.done,None

        
    def step(self,action):
        
        self.iter_step(10)
        
        obs = None
        reward = None
        done = None
        info = None
        
        return obs,reward,done,info

    def iter_step(self,n_steps:int):
        os.system(f'gz world --multi-step={n_steps}')

    def reset(self):

        ## RESET REWARD CALC VALUES
        self.done = False
        self.d_min = 50.0  # Reset max from ceiling [m]

        ## DISABLE STICKY LEGS (ALSO BREAKS CURRENT CONNECTION JOINTS)
        self.SendCmd('Tumble',cmd_flag=0)
        self.SendCmd('StickyPads',cmd_flag=0)

        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.sleep(0.01)

        self.SendCmd('Tumble',cmd_flag=1)
        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.sleep(1.0)

        self.gazebo_pause_physics()

        ## RESET STATE
        vel = np.random.uniform(low=1.5,high=3.5)
        phi = np.random.uniform(low=30,high=90)
        phi = 70

        vx_0 = vel*np.cos(np.deg2rad(phi))
        vz_0 = vel*np.sin(np.deg2rad(phi))

        ## RESET OBSERVATION
        Tau_0 = 0.5
        d_ceil_0 = Tau_0*vz_0 + 1e-3
        OFy = -vx_0/d_ceil_0
        self.obs = (Tau_0,OFy,d_ceil_0)

        z_0 = self.h_ceiling - d_ceil_0
        x_0 = 0.0
        self.Vel_Launch([x_0,0.0,z_0],[vx_0,0,vz_0])

        return np.array(self.obs,dtype=np.float32)


    def ParamOptim_reset(self):

        ## RESET REWARD CALC VALUES
        self.done = False
        self.d_min = 50.0  # Reset max from ceiling [m]

        ## DISABLE STICKY LEGS (ALSO BREAKS CURRENT CONNECTION JOINTS)
        self.SendCmd('Tumble',cmd_flag=0)
        self.SendCmd('StickyPads',cmd_flag=0)

        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.sleep(0.01)

        self.SendCmd('Tumble',cmd_flag=1)
        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.sleep(1.0)


        obs = None
        return obs

    def CalcReward(self):

        ## PAD CONTACT REWARD
        if self.pad_connections >= 3: 
            if self.BodyContact_flag == False:
                R1 = 0.7
            else:
                R1 = 0.4
        elif self.pad_connections == 2: 
            if self.BodyContact_flag == False:
                R1 = 0.2
            else:
                R1 = 0.1
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

        self.callService('/CF_DC/Cmd_CF_DC',srv,RLCmd)
        
    def callService(self,addr,srv,srv_type,retries=5):

        FailureModes = self.diagnosticTest()

        if any(FailureModes):
            self.Restart(FailureModes)
            self.done = True

        for retry in range(retries):
            try:
                service = rospy.ServiceProxy(addr, srv_type)
                service(srv)
                return True

            except rospy.ServiceException as e:
                print(f"[WARNING] {addr} service call failed (callService)")
                print(f"[WARNING] {e}")
                FailureModes[2] = False

                self.Restart(FailureModes)
                self.done = True
        
        return False

    def diagnosticTest(self):
        FailureModes = [False,False,False]
        sys.stdout.write(YELLOW)
        
        ## CHECK THAT GAZEBO IS FUNCTIONING
        try:
            rospy.wait_for_service("/gazebo/pause_physics",timeout=1)
        except rospy.ROSException as e:
            print(f"[WARNING] /gazebo/pause_physics wait for service failed (callService)")
            print(f"[WARNING] {e}")
            FailureModes[0] = True

        ## CHECK THAT CONTROLLER IS FUNCTIONING
        try:
            rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=1)
        except rospy.ROSException as e:
            print(f"[WARNING] /CTRL/Cmd_ctrl wait for service failed (callService)")
            print(f"[WARNING] {e}")
            FailureModes[1] = True

        ## CHECK THAT CF_DC IS FUNCTIONING
        try:
            rospy.wait_for_service('/CF_DC/Cmd_CF_DC',timeout=1)
        except rospy.ROSException as e:
            print(f"[WARNING] /CF_DC/Cmd_CF_DC wait for service failed (callService)")
            print(f"[WARNING] {e}")
            FailureModes[2] = True

        sys.stdout.write(ENDC)
        return FailureModes

    def Restart(self,FailureModes=[True,False,False]):
        sys.stdout.write(YELLOW)

        if FailureModes == [True,False,False]:
            for retry in range(3):
                try:
                    print(f"[WARNING] Gazebo Restart. Attempt: {retry+1}/3 (restart_Gazebo)")
                    self.launch_Gazebo()
                    rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
                    print(GREEN+ f"[WARNING] Gazebo restart successful. (restart_Gazebo)" + ENDC)
                    return True

                except rospy.ROSException as e:
                    print(f"[WARNING] Gazebo restart Failed. (restart_Gazebo)")
                    print(f"[WARNING] {e}")
            
        sys.stdout.write(RED)
        for retry in range(5):
            try:
                print(f"[WARNING] Full Simulation Restart. Attempt: {retry+1}/5 (Restart Sim)")

                ## KILL EVERYTHING

                os.system("killall -9 gzserver gzclient")
                os.system("rosnode kill /gazebo /gazebo_gui /Controller_Node /CF_DataConverter_Node")
                time.sleep(2.0)
                
                subprocess.Popen( # Gazebo Process
                    "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
                    start_new_session=True, shell=True)
                rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
        
                ## LAUNCH CONTROLLER
                subprocess.Popen( # Controller Process
                    "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_control controller",
                    close_fds=True, preexec_fn=os.setsid, shell=True)
                rospy.wait_for_service("/CTRL/Cmd_ctrl",timeout=5)

                ## LAUNCH CF_DC
                subprocess.Popen( # CF_DC Process
                    "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_data_converter CF_DataConverter",
                    close_fds=True, preexec_fn=os.setsid, shell=True)
                rospy.wait_for_service("/CF_DC/Cmd_CF_DC",timeout=5)

                ## UNPAUSE GAZEBO
                rospy.wait_for_service("/gazebo/pause_physics",timeout=10)
                print(GREEN + f"[WARNING] Gazebo restart successful. (Restart Sim)" + ENDC)

                return True

            except rospy.ROSException as e:
                print(f"[WARNING] Gazebo restart Failed. (Restart Sim)")
                print(f"[WARNING] {e}")

            print('\x1b[0m')

    def sleep(self,time_s):
        """
        Sleep in terms of Gazebo sim seconds not real time seconds
        """

        t_start = self.t
        while self.t - t_start <= time_s:
            # print("Sleeping....")
            FailureModes = self.diagnosticTest()
            if any(FailureModes):
                self.Restart(FailureModes)
                self.done = True
                return False

            ## NEGATIVE TIME DELTA
            if self.t < t_start:
                self.done = True
                return False

        return True
        
    def Vel_Launch(self,pos_0,vel_d,quat_0=[0,0,0,1]): 
        """Launch crazyflie from the specified position/orientation with an imparted velocity.
        NOTE: Due to controller dynamics, the actual velocity will NOT be exactly the desired velocity

        Args:
            pos_0 (list): Launch position [m]   | [x,y,z]
            vel_d (list): Launch velocity [m/s] | [Vx,Vy,Vz]
            quat_0 (list, optional): Orientation at launch. Defaults to [0,0,0,1].
        """        

        ## SET DESIRED VEL IN CONTROLLER
        self.SendCmd('Pos',cmd_flag=0)
        self.iter_step(4)
        self.SendCmd('Vel',cmd_vals=vel_d,cmd_flag=1)
        self.iter_step(4)

        ## CREATE SERVICE MESSAGE
        state_srv = ModelState()
        state_srv.model_name = self.modelName

        ## INPUT POSITION AND ORIENTATION
        state_srv.pose.position.x = pos_0[0]
        state_srv.pose.position.y = pos_0[1]
        state_srv.pose.position.z = pos_0[2]

        state_srv.pose.orientation.x = quat_0[0]
        state_srv.pose.orientation.y = quat_0[1]
        state_srv.pose.orientation.z = quat_0[2]
        state_srv.pose.orientation.w = quat_0[3]

        ## INPUT LINEAR AND ANGULAR VELOCITY
        state_srv.twist.linear.x = vel_d[0]
        state_srv.twist.linear.y = vel_d[1]
        state_srv.twist.linear.z = vel_d[2]

        state_srv.twist.angular.x = 0
        state_srv.twist.angular.y = 0
        state_srv.twist.angular.z = 0

        ## PUBLISH MODEL STATE SERVICE REQUEST
        self.callService('/gazebo/set_model_state',state_srv,SetModelState)

                
    def reset_pos(self,z_0=0.379): # Disable sticky then places spawn_model at origin
        """Reset pose/twist of simulated drone back to home position. 
        As well as turning off stickyfeet

        Args:
            z_0 (float, optional): Starting height of crazyflie. Defaults to 0.379.
        """        
        
        ## RESET POSITION AND VELOCITY
        state_srv = ModelState()
        state_srv.model_name = self.modelName
        state_srv.pose.position.x = 0.0
        state_srv.pose.position.y = 0.0
        state_srv.pose.position.z = z_0

        state_srv.pose.orientation.w = 1.0
        state_srv.pose.orientation.x = 0.0
        state_srv.pose.orientation.y = 0.0
        state_srv.pose.orientation.z = 0.0
        
        state_srv.twist.linear.x = 0.0
        state_srv.twist.linear.y = 0.0
        state_srv.twist.linear.z = 0.0

        state_srv.twist.angular.x = 0.0
        state_srv.twist.angular.y = 0.0
        state_srv.twist.angular.z = 0.0

        self.callService('/gazebo/set_model_state',state_srv,SetModelState)


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
        term_command = "rosnode kill /gazebo /gazebo_gui"
        os.system(term_command)
        time.sleep(1.0)

        term_command = "killall -9 gzserver gzclient"
        os.system(term_command)
        time.sleep(1.0)
        
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

       

    def gazebo_pause_physics(self,retries=5) -> bool:
        """
        Function to pause the physics in the simulation.
        :param retries: The number of times to retry the service call.
        :type retries: int
        :return: True if the command was sent and False otherwise.
        :rtype: bool
        """

        rospy.wait_for_service("/gazebo/pause_physics",timeout=5)
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

        rospy.wait_for_service("/gazebo/unpause_physics",timeout=5)
        client_srv = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        
        for retry in range(retries):
            try:
                client_srv()
                return True
            except rospy.ServiceException as e:
                print ("/gazebo/pause_physics service call failed")
            
        return False
        
    def preInit_Values(self):
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

    # env = CrazyflieEnv(gazeboTimeout=True)

    # for ii in range(1000):
    #     tau = np.random.uniform(low=0.15,high=0.27)
    #     env.ParamOptim_reset()
    #     obs,reward,done,info = env.ParamOptim_Flight(0.23,7,2.5,60)
    #     print(f"Ep: {ii} \t Reward: {reward:.02f}")

    env = CrazyflieEnv(gazeboTimeout=True)
    for _ in range(25):
        env.reset()
        done = False
        env.SendCmd("Policy",[0.25,7,0],cmd_flag=1)
        while not done:
            obs,reward,done,info = env.step(1)

