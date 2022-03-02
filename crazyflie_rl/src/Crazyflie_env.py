import numpy as np
from threading import Thread, Timer

import time, csv
import os, subprocess, signal
import rospy
import getpass


from std_srvs.srv import Empty



from sensor_msgs.msg import LaserScan, Image, Imu
from crazyflie_msgs.msg import RLData,RLCmd,RLConvg
from crazyflie_msgs.msg import ImpactData,CtrlData,PadConnect
from crazyflie_msgs.srv import activateSticky

from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState,ContactsState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry



class CrazyflieEnv:
    def __init__(self,gazeboTimeout=True):
        print("[STARTING] CrazyflieEnv is starting...")


        ## GAZEBO SIMULATION INITIALIZATION
        rospy.init_node("crazyflie_env_node") 
        os.system("roslaunch crazyflie_launch params.launch")
        # self.launch_controller()
        self.launch_sim() 
        rospy.wait_for_message("/clock",Clock)
        print("[INITIATING] Gazebo simulation started")


        self.username = getpass.getuser()
        self.loggingPath =  f"/home/{self.username}/catkin_ws/src/crazyflie_simulation/crazyflie_logging/local_logs"
        self.dataType = "SIM"
        self.filepath = ""
        self.trial_name = '' 
        self.error_str = ''     # Label for why rollout was terminated/completed
        self.agent_name = ''    # Learning agent used for training (PEPG,EM,etc...)
        self.logging_flag = True
        self.runComplete_flag = False
        self.trialComplete_flag = False
        self.reset_flag = False




        ## LOAD SIM_SETTINGS/ROS_PARAMETERS
        self.modelName = rospy.get_param('/MODEL_NAME')
        self.modelInitials = self.modelInitial()
        
        self.t_start = rospy.get_time()

        ## RAW VICON VALUES
        self.posViconRaw = [0,0,0]
        self.quatViconRaw = [0,0,0,1]

        ## FILTERED VICON STATES
        self.posVicon = [0,0,0]
        self.velVicon = [0,0,0]

        self.quatVicon = [0,0,0,1]
        self.eulVicon = [0,0,0]
        self.omegaVicon = [0,0,0]

        ## CRAZYFLIE STATES
        self.posCF = [0,0,0]
        self.velCF = [0,0,0]

        self.quatCF = [0,0,0,1]
        self.eulCF = [0,0,0]
        self.omegaCF = [0,0,0]

        self.x_d = [0,0,0]
        self.v_d = [0,0,0]
        self.a_d = [0,0,0]

        self.MS_pwm = [0,0,0,0] # Controller Motor Speeds (MS1,MS2,MS3,MS4) [PWM]
        self.FM = [0,0,0,0]     # Controller Force/Moments (F_thrust,Mx,My,Mz) [N,N*mm]
        self.V_Battery = 0.0
        self.flip_flag = False      # Flag if model has started flip maneuver

        self.Tau = 0.0
        self.OFx = 0.0
        self.OFy = 0.0
        self.RREV = 0.0
        self.d_ceil = 0.0 

        self.NN_flip = 0.0
        self.NN_policy = 0.0

        ## INIT RL PARAMETERS
        self.n_rollouts = 0     # Rollouts per episode
        self.h_ceiling = rospy.get_param("/CEILING_HEIGHT") # [m]

        self.k_ep = 0           # Episode number
        self.k_run = 0          # Run number

        self.mu = []            # Gaussian mean that policies are sampled from
        self.sigma = []         # Gaussian standard deviation policies are sampled from
        self.policy = []        # Policy sampled from Gaussian distribution

        self.mu_1_list = []
        self.mu_2_list = []

        self.sigma_1_list = []
        self.sigma_2_list = []

        self.reward = 0.0       # Calculated reward from run
        self.reward_avg = 0.0   # Averaged rewards over episode
        self.reward_inputs = [] # List of inputs to reward func

        self.z_max = 0.0
        self.pitch_sum = 0.0
        self.pitch_max = 0.0

        self.vel_trial = [0.0,0.0,0.0] # Desired velocity for trial
     
        ## TRAJECTORY VALUES
        self.posCF_0 = [0,0,0.4] # Default hover position [m]
        self.accCF_max = [2.0,2.0,4.0] # Max acceleration values for trajectory generation [m/s^2]
        
      


        self.t_prev = 0.0       # [s]

        ## INITIALIZE FLIP VALUES
        self.t_tr = 0.0             # [s]
        self.posCF_tr = [0,0,0]     # [m]
        self.velCF_tr = [0,0,0]     # [m/s]
        self.quatCF_tr = [0,0,0,1]  # [quat]
        self.omegaCF_tr = [0,0,0]   # [rad/s]

        self.Tau_tr = 0.0
        self.RREV_tr = 0.0          # [rad/s]
        self.OFx_tr = 0.0           # [rad/s]
        self.OFy_tr = 0.0           # [rad/s]
        self.d_ceil_tr = 0.0        # [m]

        self.FM_tr = [0,0,0,0]      # [N,N*mm]

        self.NN_tr_flip = 0.0
        self.NN_tr_policy = 0.0     # [N*mm]

        

        ## INITIALIZE IMPACT VALUES
        self.t_impact = np.nan
        self.posCF_impact = [np.nan,np.nan,np.nan]
        self.velCF_impact = [np.nan,np.nan,np.nan]
        self.quatCF_impact = [np.nan,np.nan,np.nan,np.nan]
        self.omegaCF_impact = [np.nan,np.nan,np.nan]

        self.FM_impact = [0,0,0,0]  # [N,N*mm]

        self.pad_contacts = 0       # Flag if pads impact ceiling plane
        self.body_contact = False   # Flag if model body impacts ceiling plane
        self.impact_flag = False    # Flag if any model part impact ceiling plane

        self.ceiling_ft_x = np.nan     # Ceiling impact force, X-dir [N]
        self.ceiling_ft_y = np.nan     # Ceiling impact force, Y-dir [N]
        self.ceiling_ft_z = np.nan     # Ceiling impact force, Z-dir [N]



        ## INIT ROS SUBSCRIBERS [Pub/Sampling Frequencies]
        # NOTE: Queue sizes=1 so that we are always looking at the most current data and 
        #       not data at back of a queue waiting to be processed by callbacks

        ## CONTROLLER TOPICS 
        self.CTRL_Data_Subscriber = rospy.Subscriber('/CTRL/data',CtrlData,self.ctrlData_Callback,queue_size=1)                                   

        ## RL TOPICS 
        self.RL_Data_Publisher = rospy.Publisher('/RL/data',RLData,queue_size=10)
        self.RL_CMD_Publisher = rospy.Publisher('/RL/cmd',RLCmd,queue_size=10)
        self.RL_Convg_Publisher = rospy.Publisher('/RL/convg_data',RLConvg,queue_size=10)

        ## INTERNAL TOPICS
        self.CF_OF_Subscriber = rospy.Subscriber('/CF_Internal/OF_sensor',Odometry,self.OFsensor_Callback,queue_size=1)    
        self.CF_Laser_Subscriber = rospy.Subscriber('/CF_Internal/Laser_sensor',LaserScan,self.laser_sensorCallback)       

        ## ENVIRONMENT TOPICS
        self.ENV_Clock_Subscriber = rospy.Subscriber("/clock",Clock,self.clock_Callback,queue_size=1)
        self.ENV_Vicon_Subscriber = rospy.Subscriber('/ENV/viconState_UKF',Odometry,self.viconState_Callback,queue_size=1)      
        self.ENV_BodyContact_Subscriber = rospy.Subscriber('/ENV/BodyContact',ContactsState,self.contactSensorCallback,queue_size=10)     
        self.ENV_PadConnect_Subcriber = rospy.Subscriber('/ENV/Pad_Connections',PadConnect,self.padConnect_Callback,queue_size=10)       
        # self.ENV_Ceiling_FT_Subscriber = rospy.Subscriber('/CF_DC/ImpactData',ImpactData,self.ceiling_ftsensorCallback,queue_size=10) 

        # rospy.wait_for_message('/ctrl_data',CtrlData) # Wait to receive ctrl pub to run before continuing



        ## INIT GAZEBO TIMEOUT THREAD
        if gazeboTimeout==True:
            self.timeoutThread = Thread(target=self.timeoutSub)
            self.timeoutThread.start()

        print("[COMPLETED] Environment done")

    

    def modelInitial(self): # RETURNS INITIALS FOR MODEL
        str = self.modelName
        charA = str[self.modelName.find("_")+1] # [W]ide
        charB = str[self.modelName.find("-")+1] # [L]ong

        return charA+charB  # [WL]


    # ============================
    ##   Publishers/Subscribers 
    # ============================

    

    def RL_Publish(self):
        # Publishes all the RL data from the RL script
        
        rl_msg = RLData() ## Initialize RLData message
        
        rl_msg.trial_name = self.trial_name
        rl_msg.agent = self.agent_name
        rl_msg.error = self.error_str
        rl_msg.impact_flag = self.impact_flag
        rl_msg.runComplete_flag = self.runComplete_flag
        rl_msg.trialComplete_flag = self.trialComplete_flag
        rl_msg.reset_flag = self.reset_flag


        rl_msg.n_rollouts = self.n_rollouts
        rl_msg.h_ceiling = self.h_ceiling

        rl_msg.k_ep = self.k_ep
        rl_msg.k_run = self.k_run

        rl_msg.mu = self.mu
        rl_msg.sigma = self.sigma
        rl_msg.policy = self.policy

        rl_msg.reward = self.reward
        rl_msg.reward_avg = self.reward_avg

        rl_msg.vel_d = self.vel_trial
        # rl_msg.M_d = self.M_d
        rl_msg.leg_contacts = self.pad_contacts
        rl_msg.body_contact = self.body_contact
        self.RL_Data_Publisher.publish(rl_msg) ## Publish RLData message

        rl_convg_msg = RLConvg()
        rl_convg_msg.mu_1_list = self.mu_1_list
        rl_convg_msg.mu_2_list = self.mu_2_list
        rl_convg_msg.sigma_1_list = self.sigma_1_list
        rl_convg_msg.sigma_2_list = self.sigma_2_list
        self.RL_Convg_Publisher.publish(rl_convg_msg) ## Publish RLData message


        


    def ctrlData_Callback(self,ctrl_msg): ## Callback to parse data received from controller
        
        # ## SET & TRIM CTRL VALUES FROM CTRL_DATA TOPIC

        # self.FM = np.asarray(ctrl_msg.FM)   # Force/Moments [N,N*mm]
        # self.FM = np.round(self.FM,3)       # Round data for logging
        # self.MS_pwm = np.asarray(ctrl_msg.MS_PWM)
        # self.MS_pwm = np.round(self.MS_pwm,0)

        # self.NN_flip = np.round(ctrl_msg.NN_flip,3)
        # self.NN_policy = np.round(ctrl_msg.NN_policy,3)


        
        # # if ctrl_msg.flip_flag == True and self.flip_flag == False: # Activates only once per run when flip_flag is about to change

        # self.flip_flag = ctrl_msg.flip_flag # Update flip_flag

        # # Save state data at time of flip activation
        # ## SET STATE VALUES FROM TOPIC
        # # TIME_FLIP
        # # t_temp = ft_msg.Header.stamp.secs
        # # ns_temp = ft_msg.Header.stamp.nsecs
        # # self.t_impact = np.round(t_temp+ns_temp*1e-9,4)  

        # t_temp = ctrl_msg.Pose_tr.header.stamp.secs
        # ns_temp = ctrl_msg.Pose_tr.header.stamp.nsecs
        # self.t_tr = np.round(t_temp+ns_temp*1e-9,4)  # Treat nsecs here at micro-secs

        # # POSE_FLIP
        # self.posCF_tr = np.round([ctrl_msg.Pose_tr.pose.position.x,
        #                             ctrl_msg.Pose_tr.pose.position.y,
        #                             ctrl_msg.Pose_tr.pose.position.z],3) # [m]
        # self.quatCF_tr = np.round([ctrl_msg.Pose_tr.pose.orientation.x,
        #                             ctrl_msg.Pose_tr.pose.orientation.y,
        #                             ctrl_msg.Pose_tr.pose.orientation.z,
        #                             ctrl_msg.Pose_tr.pose.orientation.w],5) # [quat]
        # # TWIST_FLIP
        # self.velCF_tr = np.round([ctrl_msg.Twist_tr.linear.x,
        #                             ctrl_msg.Twist_tr.linear.y,
        #                             ctrl_msg.Twist_tr.linear.z],3) # [m/s]
        # self.omegaCF_tr = np.round([ctrl_msg.Twist_tr.angular.x,
        #                             ctrl_msg.Twist_tr.angular.y,
        #                             ctrl_msg.Twist_tr.angular.z],3) # [rad/s]


        # self.FM_tr = np.asarray(ctrl_msg.FM_flip) # Force/Moments [N,N*mm]
        # self.FM_tr = np.round(self.FM_tr,3)
        
        # self.RREV_tr = np.round(ctrl_msg.RREV_tr,3) # Recorded trigger RREV [rad/s]
        # self.OFy_tr = np.round(ctrl_msg.OFy_tr,3) # Recorded OFy at trigger [rad/s]
        # self.OFx_tr = np.round(ctrl_msg.OFy_tr,3) # Recorded OF_x at trigger [rad/s]

        # self.NN_tr_flip = np.round(ctrl_msg.NN_tr_flip,3)
        # self.NN_tr_policy = np.round(ctrl_msg.NN_tr_policy,3)


        pass



    # ============================
    ##    Sensors/Data Topics
    # ============================

    def clock_Callback(self,clock_msg): ## Callback to grab time from sim clock

        self.t = np.round(clock_msg.clock.to_sec(),4)    # Processed timestamp from /clock msg

    def viconState_Callback(self,msg): ## Callback to parse state data received from external pos. sensor


        t = np.round(msg.header.stamp.to_sec(),4)
        
        ## SIMPLIFY STATE VALUES FROM TOPIC
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        omega = msg.twist.twist.angular
        
        if quat.w == 0: # If zero at startup set quat.w to one to prevent errors
            quat.w = 1

        ## SET STATE VALUES FROM TOPIC
        self.posCF = np.round([pos.x,pos.y,pos.z],3)            # [m]
        self.quatCF = np.round([quat.x,quat.y,quat.z,quat.w],4) # [quat]
        self.velCF = np.round([vel.x,vel.y,vel.z],3)            # [m/s]
        self.omegaCF = np.round([omega.x,omega.y,omega.z],3)    # [rad/s]
        

        ## ======== REWARD INPUT CALCS ======== ##

        ## MAX Z CALC
        if self.posCF[2] > self.z_max:
            self.z_max = self.posCF[2]       # Max height achieved, used for reward calc
            self.z_max = np.round(self.z_max,3) # Rounded for data logging

        ## MAX PITCH CALC
        # Integrate omega_y over time to get full rotation estimate
        # This accounts for multiple revolutions that euler angles/quaternions can't
        self.pitch_sum = self.pitch_sum + self.omegaCF[1]*(180/np.pi)*(t - self.t_prev) # [deg]
        
        if self.pitch_sum < self.pitch_max:     # Recording the most negative value
            self.pitch_max = self.pitch_sum
            self.pitch_max = np.round(self.pitch_max,3)


        self.t_prev = t # Save t value for next callback iteration


    def OFsensor_Callback(self,OF_msg): ## Callback to parse state data received from mock OF sensor

        ## SET VISUAL CUE SENSOR VALUES FROM TOPIC
        self.Tau = OF_msg.Tau
        self.OFx = OF_msg.OFx
        self.OFy = OF_msg.OFy
        self.RREV = OF_msg.RREV
        self.d_ceil = OF_msg.d_ceil
        

    def contactSensorCallback(self,msg_arr): ## Callback to indicate if quadrotor body contacts ceiling

        for msg in msg_arr.states: ## ContactsState message includes an array of ContactState messages

            if msg.collision1_name == f"{self.modelName}::crazyflie_body::body_collision" and self.body_contact == False:
                self.body_contact = True


    def padConnect_Callback(self,msg): ## Callback that records number of pads that stick to ceiling (from gazebo_sticky_foot.cpp)

        # self.pad_contacts.append(msg.Pad_Num)
        pass
        

            

    def ceiling_ftsensorCallback(self,ft_msg):
        # Keeps record of max impact force for each run
        # The value is reset in the topic converter script at each 'home' command

        self.ceiling_ft_x = np.round(ft_msg.Force_impact.x,3)   # X-Impact force from ceiling Force-Torque sensor
        self.ceiling_ft_y = np.round(ft_msg.Force_impact.y,3)
        self.ceiling_ft_z = np.round(ft_msg.Force_impact.z,3)   # Z-Impact force from ceiling Force-Torque sensor

        self.impact_flag = ft_msg.impact_flag

        t_temp = ft_msg.Header.stamp.secs
        ns_temp = ft_msg.Header.stamp.nsecs
        self.t_impact = np.round(t_temp+ns_temp*1e-9,4)  

        self.posCF_impact = np.round([ft_msg.Pose_impact.position.x,
                                    ft_msg.Pose_impact.position.y,
                                    ft_msg.Pose_impact.position.z],3)

        self.quatCF_impact = np.round([
                                    ft_msg.Pose_impact.orientation.x,
                                    ft_msg.Pose_impact.orientation.y,
                                    ft_msg.Pose_impact.orientation.z,
                                    ft_msg.Pose_impact.orientation.w,],5)

        self.velCF_impact = np.round([ft_msg.Twist_impact.linear.x,
                                    ft_msg.Twist_impact.linear.y,
                                    ft_msg.Twist_impact.linear.z],3)

        self.omegaCF_impact = np.round([ft_msg.Twist_impact.angular.x,
                                    ft_msg.Twist_impact.angular.y,
                                    ft_msg.Twist_impact.angular.z],3)

        #     self.FM_impact = self.FM
            

    def laser_sensorCallback(self,data): # callback function for laser subsriber (Not fully implemented yet)
        self.laser_msg = data
        if  self.laser_msg.ranges[0] == float('Inf'):
            # sets dist to 4 m if sensor reads Infinity (no walls)
            self.laser_dist = 4 # max sesnsor dist
        else:
            self.laser_dist = self.laser_msg.ranges[0]




    # ============================
    ##       Sim Operation
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
        rospy.wait_for_message('/env/global_state_data',Odometry) # Wait for global state message before resuming training

    def close_sim(self):
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)
        
    def close_controller(self):
        os.killpg(self.controller_p.pid, signal.SIGTERM)



    def close_dashboard(self):
        os.killpg(self.dashboard_p.pid, signal.SIGTERM)

    def __del__(self):
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)
        # os.killpg(self.dashboard_p.pid, signal.SIGTERM)
        # os.killpg(self.controller_p.pid, signal.SIGTERM)
     
    def getTime(self):
        return self.t

    def launch_sim(self):
        
        print("[STARTING] Starting Gazebo Process...")
        self.gazebo_p = subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory  --geometry 70x48+1050+0 -- rosrun crazyflie_launch launch_gazebo.bash", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
        
        
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
                

    def reset_pos(self): # Disable sticky then places spawn_model at origin
        
        ## TURN OFF STICKY FEET
        self.step('tumble',ctrl_flag=0) # Tumble Detection off
        self.step('sticky',ctrl_flag=0)
        self.step('home')
        
        

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

        ## WAIT FOR CONTROLLER TO UPDATE STATE x2 BEFORE TURNING ON TUMBLE DETECTION
        time.sleep(0.1)
        self.step('tumble',ctrl_flag=1) # Tumble Detection on

        # time.sleep(0.1) # Give it time for controller to receive new states
        # rospy.wait_for_service('/gazebo/get_link_state')

        ## RESET TO HOME
        


    def step(self,action,ctrl_vals=[0,0,0],ctrl_flag=1):

        if action == "sticky":

            rospy.wait_for_service("/activate_Sticky_Pad_1")
            if ctrl_flag == 1: 
                for ii in range(4):
                    sticky_srv = rospy.ServiceProxy(f"/activate_Sticky_Pad_{ii+1}", activateSticky)
                    sticky_srv(True)
                    
            elif ctrl_flag == 0:
                for ii in range(4):
                    sticky_srv = rospy.ServiceProxy(f"/activate_Sticky_Pad_{ii+1}", activateSticky)
                    sticky_srv(False)
                    
            # rospy.wait_for_message("/ctrl_data",CtrlData,timeout=0.5) # Ensure controller has time to process command

        cmd_msg = RLCmd()

        cmd_dict = {'home':0,
                    'pos':1,
                    'vel':2,
                    'acc':3,
                    'tumble':4,
                    'stop':5,
                    'gains':6,
                    'moment':7,
                    'policy':8,
                    'traj':9,
                    'sticky':11}
        

        cmd_msg.cmd_type = cmd_dict[action]
        cmd_msg.cmd_vals.x = ctrl_vals[0]
        cmd_msg.cmd_vals.y = ctrl_vals[1]
        cmd_msg.cmd_vals.z = ctrl_vals[2]
        cmd_msg.cmd_flag = ctrl_flag
        
        for ii in range(1):
            self.RL_CMD_Publisher.publish(cmd_msg) # For some reason it doesn't always publish
        
        time.sleep(0.05)
        
    def clear_rollout_Data(self):
        """Clears all logged impact and flip data & resets default values before next rollout
        """               

        ## RESET IMPACT CONDITIONS
        self.impact_flag = False
        self.pad_contacts = [] # Reset impact condition variables
        self.body_contact = False
        self.ceiling_ft_x = 0.0
        self.ceiling_ft_y = 0.0
        self.ceiling_ft_z = 0.0

        ## RESET FLIP CONDITIONS
        self.flip_flag = False


        ## RESET REWARD CALC VALUES
        self.z_max = 0.0
        self.pitch_sum = 0.0
        self.pitch_max = 0.0


        self.quatCF_impact = [0,0,0,1]



    # ============================
    ##      Data Logging 
    # ============================
    
    def create_csv(self,filepath):

        if self.logging_flag:
        
            with open(filepath,mode='w') as state_file:
                state_writer = csv.writer(state_file,delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    # Generic Labels
                    'k_ep','k_run',    
                    't',         
                    'NN_flip','NN_policy',
                    'mu','sigma', 'policy',

                    # Internal State Estimates (EKF)
                    'x','y','z',            
                    'vx','vy','vz',
                    'qx','qy','qz','qw',
                    'wx','wy','wz',

                    # Misc RL labels
                    'reward','flip_flag','impact_flag','n_rollouts', 

                    # Misc Internal State Estimates
                    'Tau','OF_x','OF_y','RREV','d_ceil',       
                    'F_thrust[N]','Mx[Nmm]','My[Nmm]','Mz[Nmm]',
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
            with open(self.filepath, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    # Generic Labels
                    self.k_ep,self.k_run,
                    self.t,
                    self.NN_flip,self.NN_policy, # alpha_mu,alpha_sig
                    "","","", # mu,sigma,policy

                    # Internal State Estimates (EKF)
                    self.posCF[0],self.posCF[1],self.posCF[2], # t,x,y,z
                    self.velCF[0],self.velCF[1],self.velCF[2], # vx,vy,vz
                    self.quatCF[0],self.quatCF[1],self.quatCF[2],self.quatCF[3], # qx,qy,qz,qw
                    self.omegaCF[0],self.omegaCF[1],self.omegaCF[2], # wx,wy,wz

                    # Misc RL labels
                    "",self.flip_flag,self.impact_flag,"", # reward, flip_triggered, impact_flag, n_rollout

                    # Misc Internal State Estimates
                    self.Tau,self.OFx,self.OFy,self.RREV,self.d_ceil, # Tau,OF_x,OF_y,RREV,d_ceil
                    self.FM[0],self.FM[1],self.FM[2],self.FM[3], # F_thrust[N],Mx[Nmm],My[Nmm],Mz[Nmm]
                    self.MS_pwm[0],self.MS_pwm[1],self.MS_pwm[2],self.MS_pwm[3],

                    # Setpoint Values
                    self.x_d[0],self.x_d[1],self.x_d[2],                                # Position Setpoints
                    self.v_d[0],self.v_d[1],self.v_d[2],                                # Velocity Setpoints
                    self.a_d[0],self.a_d[1],self.a_d[2],                                # Acceleration Setpoints
                    
                    # Misc Values
                    "",
                    error_str]) # Error

    

    def append_flip(self):
        if self.logging_flag:
    
            with open(self.filepath,mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
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
                    

                    # Misc RL labels
                    "","","","", # reward, body_impact, num leg contacts, impact force

                    # Misc Internal State Estimates
                    self.Tau_tr,self.OFx_tr,self.OFy_tr,self.RREV_tr,self.d_ceil_tr, # Tau,OFx,OFy,RREV,d_ceil
                    self.FM_tr[0],self.FM_tr[1],self.FM_tr[2],self.FM_tr[3], # F_thrust,Mx,My,Mz
                    "","","","",

                    # Setpoint Values
                    "","","",
                    "","","",
                    "","","",

                    # Misc Values
                    "",
                    "Flip Data"]) # Error
    
    def append_impact(self):
        if self.logging_flag:
    
            with open(self.filepath,mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
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
                    
                    # Misc RL labels
                    self.impact_flag,self.body_contact,np.array(self.pad_contacts),"",  # "", "", body_impact flag, num leg contacts, ""
                    
                    # Misc Internal State Estimates
                    self.ceiling_ft_z,self.ceiling_ft_x,self.ceiling_ft_y,"","",             # Max impact force [z], =Max impact force [x], ""
                    "","","","", # F_thrust,Mx,My,Mz (Impact)
                    "","","","", # M_pwm
                    
                    # Setpoint Values
                    "","","",
                    "","","",
                    "","","",

                    # Misc Values
                    "",
                    "Impact Data"]) # Error

    def append_IC(self):
        if self.logging_flag:
    
            with open(self.filepath,mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([

                    # Generic Labels
                    self.k_ep,self.k_run,
                    "",             # t
                    "","", 
                    np.round(self.mu,2),np.round(self.sigma,2),np.round(self.policy,2), # mu,sigma,policy

                    # Internal State Estimates (EKF)
                    "","","",       # x,y,z
                    np.round(self.vel_trial[0],2),np.round(self.vel_trial[1],2),np.round(self.vel_trial[2],2), # vx_d,vy_d,vz_d
                    "","","","",    # qx,qy,qz,qw
                    "","","",       # wx,wy,wz

                    # Misc RL labels
                    np.round(self.reward,2),np.round(self.reward_inputs,3),"",self.n_rollouts, # reward, 

                    # Misc Internal State Estimates
                    "","","","","",    # Tau,OFx,OFy,RREV,d_ceil
                    "","","","",    # F_thrust,Mx,My,Mz 
                    "","","","",    # M_pwm

                    # Setpoint Values
                    "","","",
                    "","","",
                    "","","",

                    # Misc Values
                    "",
                    self.error_str])    # Error

    def append_csv_blank(self):
        if self.logging_flag:
            with open(self.filepath, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([])

   

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

    # ============================
    ##  Control Playground Func. 
    # ============================

    