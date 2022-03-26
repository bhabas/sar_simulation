#!/usr/bin/env python3
import numpy as np
from threading import Thread, Timer

import time
import sys
import csv
import os
import subprocess
import signal
import rospy
import getpass


from std_srvs.srv import Empty
from crazyflie_msgs.msg import RLData,RLCmd,RLConvg
from crazyflie_msgs.msg import CtrlData
from crazyflie_msgs.msg import CF_StateData,CF_FlipData,CF_ImpactData,CF_MiscData
from crazyflie_msgs.srv import activateSticky

from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState,ContactsState
from gazebo_msgs.srv import SetModelState



class CrazyflieEnv:
    def __init__(self,gazeboTimeout=True):
        print("[STARTING] CrazyflieEnv is starting...")

        ## GAZEBO SIMULATION INITIALIZATION
        # Load params -> Launch sim -> Wait for sim running -> Launch controller
        rospy.init_node("crazyflie_env_node") 
        os.system("roslaunch crazyflie_launch params.launch") 
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
        self.dataType = "SIM"
        self.filepath = ""
        self.trial_name = '' 
        self.error_str = ''     # Label for why rollout was terminated/completed
        self.agent_name = ''    # Learning agent used for training (PEPG,EM,etc...)
        self.logging_flag = False
        self.runComplete_flag = False
        self.repeat_run = False



        ## LOAD SIM_SETTINGS/ROS_PARAMETERS
        self.modelName = rospy.get_param('/MODEL_NAME')
        self.modelInitials = self.modelInitials()
        
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
        self.RREV = 0.0
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

        

        ## INIT RL PARAMETERS
        self.h_ceiling = rospy.get_param("/CEILING_HEIGHT") # [m]
        self.n_rollouts = 0     # Rollouts per episode

        self.k_ep = 0           # Episode number
        self.k_run = 0          # Run number

        self.mu = []            # Gaussian mean that policies are sampled from
        self.sigma = []         # Gaussian standard deviation policies are sampled from
        self.policy = []        # Policy sampled from Gaussian distribution

        self.mu_1_list = []
        self.mu_2_list = []

        self.sigma_1_list = []
        self.sigma_2_list = []

        self.reward_list = []
        self.reward_avg_list = []

        self.reward = 0.0       # Calculated reward from run
        self.reward_avg = 0.0   # Averaged rewards over episode
        self.reward_inputs = [] # List of inputs to reward func

        self.d_ceil_min = 50.0
        self.pitch_sum = 0.0
        self.pitch_max = 0.0

        self.vel_d = [0.0,0.0,0.0] # Desired velocity for trial
     
        ## TRAJECTORY VALUES
        self.posCF_0 = [0,0,0.4]        # Default hover position [m]
        self.accCF_max = [2.0,2.0,4.0]  # Max acceleration values for trajectory generation [m/s^2]
        
      





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
        self.RREV = np.round(StateData_msg.RREV,3)
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
        if 0.1 < self.d_ceil < self.d_ceil_min:
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

        self.V_Battery = np.round(MiscData_msg.battery_voltage,4)
    

    def RL_Publish(self):
        # Publishes all the RL data from the RL script
        
        RL_msg = RLData() ## Initialize RLData message
        
        RL_msg.trial_name = self.trial_name
        RL_msg.agent = self.agent_name
        RL_msg.error = self.error_str

        RL_msg.n_rollouts = self.n_rollouts
        RL_msg.h_ceiling = self.h_ceiling

        RL_msg.k_ep = self.k_ep
        RL_msg.k_run = self.k_run

        RL_msg.mu = self.mu
        RL_msg.sigma = self.sigma
        RL_msg.policy = self.policy

        RL_msg.reward = self.reward
        RL_msg.reward_avg = self.reward_avg

        RL_msg.vel_d = self.vel_d
        RL_msg.impact_flag = self.impact_flag
        RL_msg.body_contact = self.BodyContact_flag
        RL_msg.leg_contacts = self.pad_connections
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


    def modelInitials(self): # RETURNS INITIALS FOR MODEL
        str = self.modelName
        charA = str[self.modelName.find("_")+1] # [W]ide
        charB = str[self.modelName.find("-")+1] # [L]ong

        return charA+charB  # [WL]

    def userInput(self,input_string,dataType=float):

        while True:
            try:
                vals = [dataType(i) for i in input(input_string).split(',')]
            except:
                continue
        
            if len(vals) == 1:
                return vals[0]
            else:
                return vals

    def getTime(self):
        
        return self.t

    def impactEstimate(self,posCF_0,vel_d):

        t_impact = (self.h_ceiling - posCF_0[2])/vel_d[2]

        x_impact = posCF_0[0] + vel_d[0]*t_impact
        y_impact = posCF_0[1] + vel_d[1]*t_impact
        z_impact = posCF_0[2] + vel_d[2]*t_impact

        return [x_impact,y_impact,z_impact]


    def step(self,action,cmd_vals=[0,0,0],cmd_flag=1):

        cmd_msg = RLCmd()

        cmd_dict = {'home':0,
                    'pos':1,
                    'vel':2,
                    'acc':3,
                    'tumble':4,
                    'stop':5,
                    'params':6,
                    'moment':7,
                    'policy':8,
                    'traj':9,
                    'thrusts':10,
                    'sticky':11,
                    'M_PWM':12}
        

        cmd_msg.cmd_type = cmd_dict[action]
        cmd_msg.cmd_vals.x = cmd_vals[0]
        cmd_msg.cmd_vals.y = cmd_vals[1]
        cmd_msg.cmd_vals.z = cmd_vals[2]
        cmd_msg.cmd_flag = cmd_flag
        
        self.RL_CMD_Publisher.publish(cmd_msg)
        
    def reset_reward_terms(self):

        ## RESET REWARD CALC VALUES
        self.d_ceil_min = 50.0
        self.pitch_sum = 0.0
        self.pitch_max = 0.0

    

    # ============================
    ##       GAZEBO OPERATION
    # ============================
    

    def relaunch_sim(self):
        """
            Relaunches Gazebo and resets model position but doesn't touch controller node
        """        
        self.close_sim()
        self.close_controller()
        time.sleep(5.0)
        self.launch_controller()
        self.launch_sim()

        self.reset_pos()
        # rospy.wait_for_message('/env/global_state_data',Odometry) # Wait for global state message before resuming training

    def close_sim(self):
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)
        
    def close_controller(self):
        os.killpg(self.controller_p.pid, signal.SIGTERM)

    def close_dashboard(self):
        os.killpg(self.dashboard_p.pid, signal.SIGTERM)


    def close_proc(self):
        os.system("killall gzserver gzclient")
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)
        os.killpg(self.controller_p.pid, signal.SIGTERM)
        sys.exit(0)
    
    def launch_sim(self):
        
        print("[STARTING] Starting Gazebo Process...")
        self.gazebo_p = subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
            start_new_session=True, shell=True)
  
    def pause_sim(self,pause_flag):

        if pause_flag:
            rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        else:
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)


    def launch_dashboard(self):
        print("[STARTING] Starting Dashboard...")
        self.dashboard_p = subprocess.Popen(
            "gnome-terminal -- roslaunch crazyflie_launch dashboard.launch",
            close_fds=True, preexec_fn=os.setsid, shell=True)
    
    def launch_controller(self):
        print("[STARTING] Starting Controller Process...")
        os.system("rosnode kill /controller_node")
        self.controller_p = subprocess.Popen( # Controller Process
            "roslaunch crazyflie_launch controller.launch",
            close_fds=True, preexec_fn=os.setsid, shell=True)


    def launch_IC(self,pos_z,vx_d,vz_d): # Imparts desired velocity to model (should retain current position)
        
        ## SET POSE AND TWIST OF MODEL
        state_msg = ModelState()
        state_msg.model_name = self.modelName
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0.0
        state_msg.pose.position.z = pos_z

        state_msg.pose.orientation.x = 0.0
        state_msg.pose.orientation.y = 0.0
        state_msg.pose.orientation.z = 0.0
        state_msg.pose.orientation.w = 1.0

        state_msg.twist.linear.x = vx_d
        state_msg.twist.linear.y = 0.0
        state_msg.twist.linear.z = vz_d

        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state_srv(state_msg)

    def traj_launch(self,pos_0,vel_d,quat_0=[0,0,0,1]): ## LAUNCH MODEL AT DESIRED VEL TRAJECTORY

        ## SET DESIRED VEL IN CONTROLLER
        self.step('pos',cmd_flag=0)
        self.step('vel',cmd_vals=vel_d,cmd_flag=1)

        ## CREATE SERVICE MESSAGE
        state_msg = ModelState()
        state_msg.model_name = self.modelName

        ## POS AND QUAT
        state_msg.pose.position.x = pos_0[0]
        state_msg.pose.position.y = pos_0[1]
        state_msg.pose.position.z = pos_0[2]

        state_msg.pose.orientation.x = quat_0[0]
        state_msg.pose.orientation.y = quat_0[1]
        state_msg.pose.orientation.z = quat_0[2]
        state_msg.pose.orientation.w = quat_0[3]

        ## LINEAR AND ANG. VEL
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
                

    def reset_pos(self): # Disable sticky then places spawn_model at origin

        ## DISABLE STICKY
        self.step('sticky',cmd_flag=0)
        time.sleep(0.05)
        
        ## RESET POSITION AND VELOCITY
        state_msg = ModelState()
        state_msg.model_name = self.modelName
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0.0
        state_msg.pose.position.z = 0.4

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

        ## RESET HOME/TUMBLE DETECTION AND STICKY
        self.step('tumble',cmd_flag=1) # Tumble Detection On
        self.step('home')
       


    def step(self,action,cmd_vals=[0,0,0],cmd_flag=1):

        cmd_msg = RLCmd()

        cmd_dict = {'home':0,
                    'pos':1,
                    'vel':2,
                    'acc':3,
                    'tumble':4,
                    'stop':5,
                    'params':6,
                    'moment':7,
                    'policy':8,
                    'traj':9,
                    'thrusts':10,
                    'sticky':11,
                    'M_PWM':12}
        

        cmd_msg.cmd_type = cmd_dict[action]
        cmd_msg.cmd_vals.x = cmd_vals[0]
        cmd_msg.cmd_vals.y = cmd_vals[1]
        cmd_msg.cmd_vals.z = cmd_vals[2]
        cmd_msg.cmd_flag = cmd_flag
        
        self.RL_CMD_Publisher.publish(cmd_msg) # For some reason it doesn't always publish
        
    def reset_reward_terms(self):

        ## RESET REWARD CALC VALUES
        self.d_ceil_min = 50.0
        self.pitch_sum = 0.0
        self.pitch_max = 0.0


    def impactEstimate(self,pos_0,vel_d):
        
        ## ASSUME INSTANT VELOCITY
        t_impact = (self.h_ceiling - pos_0[2])/vel_d[2]

        ## FIND IMPACT POINT FROM IMPACT TIME
        x_impact = pos_0[0] + vel_d[0]*t_impact
        y_impact = pos_0[1] + vel_d[1]*t_impact
        z_impact = pos_0[2] + vel_d[2]*t_impact

        return [x_impact,y_impact,z_impact]



    # ============================
    ##      Data Logging 
    # ============================
    
    def create_csv(self,filepath):

        if self.logging_flag:
        
            with open(filepath,mode='w') as data_file:
                data_writer = csv.writer(data_file,delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
                data_writer.writerow([
                    # Generic Labels
                    'k_ep','k_run',    
                    't',         
                    'NN_flip','NN_policy',
                    'mu','sigma', 'policy',

                    # Internal State Estimates (CF)
                    'x','y','z',            
                    'vx','vy','vz',
                    'qx','qy','qz','qw',
                    'wx','wy','wz',
                    'eul_x','eul_y','eul_z',

                    # Misc RL labels
                    'flip_flag','impact_flag', 

                    # Misc Internal State Estimates
                    'Tau','OF_x','OF_y','RREV','d_ceil',       
                    'F_thrust[N]','Mx[Nmm]','My[Nmm]','Mz[Nmm]',
                    'M1_thrust','M2_thrust','M3_thrust','M4_thrust',
                    'M1_pwm','M2_pwm','M3_pwm','M4_pwm',

                    # Setpoint Values
                    'x_d.x','x_d.y','x_d.z',    
                    'v_d.x','v_d.y','v_d.z',
                    'a_d.x','a_d.y','a_d.z',

                    # Misc Values
                    'Volts',
                    'Error'])# Place holders


    def append_csv(self,error_str= ""):

        if self.logging_flag:
            with open(self.filepath, mode='a') as data_file:
                data_write = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                data_write.writerow([
                    # Generic Labels
                    self.k_ep,self.k_run,
                    self.t,
                    self.NN_flip,self.NN_policy, # NN_flip,NN_policy
                    "","","", # mu,sigma,policy

                    # Internal State Estimates (CF)
                    self.posCF[0],self.posCF[1],self.posCF[2], # x,y,z
                    self.velCF[0],self.velCF[1],self.velCF[2], # vx,vy,vz
                    self.quatCF[0],self.quatCF[1],self.quatCF[2],self.quatCF[3], # qx,qy,qz,qw
                    self.omegaCF[0],self.omegaCF[1],self.omegaCF[2], # wx,wy,wz
                    self.eulCF[0],self.eulCF[1],self.eulCF[2], # eul_x,eul_y,eul_z


                    # Misc RL labels
                    self.flip_flag,self.impact_flag, #  flip_flag, impact_flag,

                    # Misc Internal State Estimates
                    self.Tau,self.OFx,self.OFy,self.RREV,self.d_ceil,   # Tau,OF_x,OF_y,RREV,d_ceil
                    self.FM[0],self.FM[1],self.FM[2],self.FM[3],        # F_thrust[N],Mx[Nmm],My[Nmm],Mz[Nmm]
                    self.MotorThrusts[0],self.MotorThrusts[1],self.MotorThrusts[2],self.MotorThrusts[3],
                    self.MS_pwm[0],self.MS_pwm[1],self.MS_pwm[2],self.MS_pwm[3],

                    # Setpoint Values
                    self.x_d[0],self.x_d[1],self.x_d[2], # Position Setpoints
                    self.v_d[0],self.v_d[1],self.v_d[2], # Velocity Setpoints
                    self.a_d[0],self.a_d[1],self.a_d[2], # Acceleration Setpoints
                    
                    # Misc Values
                    self.V_Battery,
                    error_str]) # Error

    def append_IC(self):
        if self.logging_flag:
    
            with open(self.filepath,mode='a') as data_file:
                data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                data_writer.writerow([

                    # Generic Labels
                    self.k_ep,self.k_run,
                    "",             # t
                    self.n_rollouts,"", 
                    np.round(self.mu,2),np.round(self.sigma,2),np.round(self.policy,2), # mu,sigma,policy

                    # Internal State Estimates (EKF)
                    "","","",       # x,y,z
                    np.round(self.vel_d[0],2),np.round(self.vel_d[1],2),np.round(self.vel_d[2],2), # vx_d,vy_d,vz_d
                    "","","","",    # qx,qy,qz,qw
                    "","","",       # wx,wy,wz
                    "","","",       # eul_x,eul_y,eul_z


                    # Misc RL labels
                    np.round(self.reward,2),np.round(self.reward_inputs,3), # reward, 

                    # Misc Internal State Estimates
                    "","","","","",    # Tau,OFx,OFy,RREV,d_ceil
                    "","","","",    # F_thrust,Mx,My,Mz 
                    "","","","",    # M_thrust [g]
                    "","","","",    # M_pwm


                    # Setpoint Values
                    "","","",
                    "","","",
                    "","","",

                    # Misc Values
                    "",
                    self.error_str])    # Error

    def append_flip(self):
        if self.logging_flag:
    
            with open(self.filepath,mode='a') as data_file:
                data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                data_writer.writerow([
                    # Generic Labels
                    self.k_ep,self.k_run,
                    self.t_tr,
                    self.NN_tr_flip,self.NN_tr_policy, # NN_flip, NN_policy
                    "","","", # mu,sigma,policy
                    
                    
                    # Internal State Estimates (EKF)
                    self.posCF_tr[0],self.posCF_tr[1],self.posCF_tr[2],    # t,x,y,z
                    self.velCF_tr[0],self.velCF_tr[1],self.velCF_tr[2],    # vx_d,vy_d,vz_d
                    self.quatCF_tr[0],self.quatCF_tr[1],self.quatCF_tr[2],self.quatCF_tr[3],    # qx,qy,qz,qw
                    self.omegaCF_tr[0],self.omegaCF_tr[1],self.omegaCF_tr[2],  # wx,wy,wz
                    self.eulCF_tr[0],self.eulCF_tr[1],self.eulCF_tr[2],     # eul_x,eul_y,eul_z

                    # Misc RL labels
                    self.flip_flag,"", # flip_flag, impact_flag

                    # Misc Internal State Estimates
                    self.Tau_tr,self.OFx_tr,self.OFy_tr,self.RREV_tr,self.d_ceil_tr, # Tau,OFx,OFy,RREV,d_ceil
                    self.FM_tr[0],self.FM_tr[1],self.FM_tr[2],self.FM_tr[3], # F_thrust,Mx,My,Mz
                    "","","","",    # M_thrust [g]
                    "","","","",    # M_pwm

                    # Setpoint Values
                    "","","",
                    "","","",
                    "","","",

                    # Misc Values
                    "",
                    "Flip Data"]) # Error
    
    def append_impact(self):
        if self.logging_flag:
    
            with open(self.filepath,mode='a') as data_file:
                data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                data_writer.writerow([
                    # Generic Labels
                    self.k_ep,self.k_run,
                    self.t_impact,
                    "","", # alpha_mu,alpha_sig
                    "","","", # mu,sigma,policy

                    # Internal State Estimates (EKF)
                    self.posCF_impact[0],self.posCF_impact[1],self.posCF_impact[2],    # t,x,y,z
                    self.velCF_impact[0],self.velCF_impact[1],self.velCF_impact[2],    # vx_d,vy_d,vz_d
                    self.quatCF_impact[0],self.quatCF_impact[1],self.quatCF_impact[2],self.quatCF_impact[3],    # qx,qy,qz,qw
                    self.omegaCF_impact[0],self.omegaCF_impact[1],self.omegaCF_impact[2],  # wx,wy,wz
                    self.eulCF_impact[0],self.eulCF_impact[1],self.eulCF_impact[2],     # eul_x,eul_y,eul_z

                    # Misc RL labels
                    self.BodyContact_flag,self.impact_flag, 
                    
                    # Misc Internal State Estimates
                    self.pad_connections,self.Pad1_Contact,self.Pad2_Contact,self.Pad3_Contact,self.Pad4_Contact, 
                    self.impact_magnitude,self.Force_impact[0],self.Force_impact[1],self.Force_impact[2], # F_thrust,Mx,My,Mz (Impact)
                    "","","","",    # M_thrust [g]
                    "","","","",    # M_pwm
                    
                    # Setpoint Values
                    "","","",
                    "","","",
                    "","","",

                    # Misc Values
                    "",
                    "Impact Data"]) # Error

    

    def append_csv_blank(self):
        if self.logging_flag:
            with open(self.filepath, mode='a') as data_file:
                data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                data_writer.writerow([])

   

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
        self.launch_controller()
        self.launch_sim()



    




if __name__ == "__main__":
    env = CrazyflieEnv()
    rospy.on_shutdown(env.close_proc)

    rospy.spin()