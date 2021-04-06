import numpy as np
from threading import Thread, Timer

import time, csv
import os, subprocess, signal
import rospy
import getpass


from sensor_msgs.msg import LaserScan, Image, Imu
from crazyflie_rl.msg import RLData,RLCmd
from crazyflie_gazebo.msg import CtrlData
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState,ContactsState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped



class CrazyflieEnv:
    def __init__(self,gazeboTimeout=True):
        print("[STARTING] CrazyflieEnv is starting...")

        self.username = getpass.getuser()
        self.loggingPath =  f"/home/{self.username}/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log"
        self.logging_flag = False
        self.filepath = ""
        self.runComplete_flag = False
        self.isRunning = True

        self.h_ceiling = 3.00 # [m]
        self.state_current = np.zeros(13)

        ## INIT NAME OF MODEL BEING USED
        self.modelName = 'crazyflie_model_Narrow-Long'

        ## INIT RL_DATA VARIABLES 
        # NOTE: All time units are in terms of Sim-Time unless specified
        #region 
        self.trial_name = '' 
        self.agent_name = ''    # Learning agent used for training (PEPG,EM,etc...)
        self.error_str = ''     # Label for why rollout was terminated/completed

        self.n_rollouts = 0     # Rollouts per episode
        
        self.k_ep = 0           # Episode number
        self.k_run = 0          # Run number

        self.alpha_mu = []      # PEPG learning rate for mu
        self.alpha_sigma = []   # PEPG learning rate for sigma

        self.mu = []            # Gaussian mean policies are sampled from
        self.sigma = []         # Gaussian standard deviation policies are sampled from
        self.policy = []        # Policy sampled from Gaussian distribution

        self.reward = 0         # Calculated reward from run
        self.reward_avg = 0     # Averaged rewards over episode
        self.reward_inputs = [] # List of inputs to reward func

        self.vel_d = [0,0,0]    # Desired/Imparted velocities for rollout [m/s]
        self.M_d = [0,0,0]      # Imparted moments for flip manuever [N*mm]

        ## INIT CTRL_DATA VARIABLES 
        self.MS = [0,0,0,0]     # Controller Motor Speeds (MS1,MS2,MS3,MS4) [rad/s]
        self.FM = [0,0,0,0]     # Controller Force/Moments (F_thrust,Mx,My,Mz) [N,N*mm]
        

        ## REWARD VALS
        self.z_max = 0.0        # Max recorded height in run [m]
        self.pitch_sum = 0.0    # Cumulative pitch angle in run [deg]
        self.pitch_max = 0.0    # Max recorded pitch angle in run [deg]
        self.t_prev = 0.0       # [s]

        ## FLIP VALS
        self.t_flip = 0.0
        self.state_flip = np.zeros(13)
        self.FM_flip = [0,0,0,0]    # [N,N*mm]

        self.flip_flag = False      # Flag if model has started flip maneuver

        ## IMPACT VALS
        self.t_impact = 0.0
        self.state_impact = np.zeros(13)
        self.FM_impact = [0,0,0,0]  # [N,N*mm]

        self.pad_contacts = [False,False,False,False] # Flag if pads impact ceiling plane
        self.body_contact = False   # Flag if model body impacts ceiling plane
        self.impact_flag = False    # Flag if any model part impact ceiling plane

        self.ceiling_ft_z = 0.0     # Ceiling impact force, Z-dir [N]
        self.ceiling_ft_x = 0.0     # Ceiling impact force, X-dir [N]
        #endregion 




        
        ## INIT ROS NODE FOR ENVIRONMENT 
        rospy.init_node("crazyflie_env_node") 
        print("[STARTING] Starting Controller Process...")
        self.controller_p = subprocess.Popen( # Controller Process
            "roslaunch crazyflie_gazebo controller.launch", close_fds=True, preexec_fn=os.setsid, shell=True)
        self.launch_sim() 
    

        

        ## INIT ROS SUBSCRIBERS [Pub/Sampling Frequencies]
        # NOTE: Queue sizes=1 so that we are always looking at the most current data and 
        #       not data at back of a queue waiting to be processed by callbacks
        self.clock_Subscriber = rospy.Subscriber("/clock",Clock,self.clockCallback,queue_size=1)
        self.state_Subscriber = rospy.Subscriber('/global_state',Odometry,self.global_stateCallback,queue_size=1)      
        self.ctrl_Subscriber = rospy.Subscriber('/ctrl_data',CtrlData,self.ctrlCallback,queue_size=1)                                   

        self.OF_Subscriber = rospy.Subscriber('/OF_sensor',Odometry,self.OFsensor_Callback,queue_size=1)                                
        self.contact_Subscriber = rospy.Subscriber('/ceiling_contact',ContactsState,self.contactSensorCallback,queue_size=50)            
        self.ceiling_ft_Subscriber = rospy.Subscriber('/ceiling_force_sensor',WrenchStamped,self.ceiling_ftsensorCallback,queue_size=50)  
        self.laser_Subscriber = rospy.Subscriber('/zranger2/scan',LaserScan,self.laser_sensorCallback)                    
        rospy.wait_for_message('/ctrl_data',CtrlData) # Wait to receive ctrl pub to run before continuing



        ## INIT ROS PUBLISHERS
        self.RL_Publisher = rospy.Publisher('/rl_data',RLData,queue_size=10)
        self.Cmd_Publisher = rospy.Publisher('/rl_ctrl',RLCmd,queue_size=10)



        ## INIT GAZEBO TIMEOUT THREAD
        if gazeboTimeout==True:
            self.timeoutThread = Thread(target=self.timeoutSub)
            self.timeoutThread.start()

        print("[COMPLETED] Environment done")

    

    

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


        rl_msg.n_rollouts = self.n_rollouts
        rl_msg.h_ceiling = self.h_ceiling

        rl_msg.k_ep = self.k_ep
        rl_msg.k_run = self.k_run

        rl_msg.alpha_mu = self.alpha_mu
        rl_msg.alpha_sigma = self.alpha_sigma
        rl_msg.mu = self.mu
        rl_msg.sigma = self.sigma
        rl_msg.policy = self.policy

        rl_msg.reward = self.reward
        rl_msg.reward_avg = self.reward_avg

        rl_msg.vel_d = self.vel_d
        rl_msg.M_d = self.M_d
        rl_msg.leg_contacts = self.pad_contacts
        rl_msg.body_contact = self.body_contact
        

        self.RL_Publisher.publish(rl_msg) ## Publish RLData message

    def ctrlCallback(self,ctrl_msg): ## Callback to parse data received from controller
        
        ## SET & TRIM CTRL VALUES FROM CTRL_DATA TOPIC
        self.MS = np.asarray(ctrl_msg.motorspeeds) # Motorspeeds [rad/s]
        self.MS = np.round(self.MS,0)

        self.FM = np.asarray(ctrl_msg.FM)   # Force/Moments [N,N*mm]
        self.FM = np.round(self.FM,3)       # Round data for logging

        
        if ctrl_msg.flip_flag == True and self.flip_flag == False: # Activates only once per run when flip_flag is about to change

            self.flip_flag = ctrl_msg.flip_flag # Update flip_flag

            # Save state data at time of flip activation
            ## SET STATE VALUES FROM TOPIC
            # TIME_FLIP
            t_temp = ctrl_msg.Pose_tr.header.stamp.secs
            ns_temp = ctrl_msg.Pose_tr.header.stamp.nsecs
            self.t_flip = np.round(t_temp+ns_temp*1e-3,3)  # Treat nsecs here at micro-secs

            # POSE_FLIP
            self.pos_flip = np.round([ctrl_msg.Pose_tr.pose.position.x,
                                        ctrl_msg.Pose_tr.pose.position.y,
                                        ctrl_msg.Pose_tr.pose.position.z],3) # [m]
            self.quat_flip = np.round([ctrl_msg.Pose_tr.pose.orientation.w,
                                        ctrl_msg.Pose_tr.pose.orientation.x,
                                        ctrl_msg.Pose_tr.pose.orientation.y,
                                        ctrl_msg.Pose_tr.pose.orientation.z],3) # [quat]
            # TWIST_FLIP
            self.vel_flip = np.round([ctrl_msg.Twist_tr.linear.x,
                                        ctrl_msg.Twist_tr.linear.y,
                                        ctrl_msg.Twist_tr.linear.z],3) # [m/s]
            self.omega_flip = np.round([ctrl_msg.Twist_tr.angular.x,
                                        ctrl_msg.Twist_tr.angular.y,
                                        ctrl_msg.Twist_tr.angular.z],3) # [rad/s]


            self.FM_flip = np.asarray(ctrl_msg.FM_flip) # Force/Moments [N,N*mm]
            self.FM_flip = np.round(self.FM_flip,3)
            
            self.RREV_tr = np.round(ctrl_msg.RREV_tr,3) # Recorded trigger RREV [rad/s]
            self.OF_y_tr = np.round(ctrl_msg.OF_y_tr,3) # Recorded OF_y at trigger [rad/s]



    # ============================
    ##    Sensors/Data Topics
    # ============================

    def clockCallback(self,clock_msg): ## Callback to grab time from Sim clock
        t_temp = clock_msg.clock.secs
        ns_temp = clock_msg.clock.nsecs

        self.t = np.round(t_temp+ns_temp*1e-9,3)

    def global_stateCallback(self,gs_msg): ## Callback to parse state data received from external pos. sensor

        t_temp = gs_msg.header.stamp.secs
        ns_temp = gs_msg.header.stamp.nsecs

        t = np.round(t_temp+ns_temp*1e-9,3)      
        
        ## SIMPLIFY STATE VALUES FROM TOPIC
        global_pos = gs_msg.pose.pose.position
        global_quat = gs_msg.pose.pose.orientation
        global_vel = gs_msg.twist.twist.linear
        global_omega = gs_msg.twist.twist.angular
        
        if global_quat.w == 0: # If zero at startup set quat.w to one to prevent errors
            global_quat.w = 1

        ## SET STATE VALUES FROM TOPIC
        self.position = np.round([global_pos.x,global_pos.y,global_pos.z],3)    # [m]
        self.orientation_q = np.round([global_quat.w,global_quat.x,global_quat.y,global_quat.z],3) # [quat]
        self.velocity = np.round([global_vel.x,global_vel.y,global_vel.z],3)    # [m/s]
        self.omega = np.round([global_omega.x,global_omega.y,global_omega.z],3) # [rad/s]


        ## COMBINE INTO COMPREHENSIVE LIST
        self.state_current = np.concatenate([self.position,self.orientation_q,self.velocity,self.omega])
        

        ## ======== REWARD INPUT CALCS ======== ##

        ## MAX Z CALC
        if self.position[2] > self.z_max:
            self.z_max = self.position[2]       # Max height achieved, used for reward calc
            self.z_max = np.round(self.z_max,3) # Rounded for data logging

        ## MAX PITCH CALC
        # Integrate omega_y over time to get full rotation estimate
        # This accounts for multiple revolutions that euler angles/quaternions can't
        self.pitch_sum = self.pitch_sum + self.omega[1]*(180/np.pi)*(t - self.t_prev) # [deg]
        
        if self.pitch_sum < self.pitch_max:     # Recording the most negative value
            self.pitch_max = self.pitch_sum
            self.pitch_max = np.round(self.pitch_max,3)


        self.t_prev = t # Save t value for next callback iteration


    def OFsensor_Callback(self,OF_msg): ## Callback to parse state data received from mock OF sensor

        ## SET VISUAL CUE SENSOR VALUES FROM TOPIC
        d = self.h_ceiling - OF_msg.pose.pose.position.z    # Distance from CF to ceiling [m]

        self.RREV = round( OF_msg.twist.twist.linear.z/d,3) # [rad/s]
        self.OF_x = round(-OF_msg.twist.twist.linear.y/d,3) # [rad/s]
        self.OF_y = round(-OF_msg.twist.twist.linear.x/d,3) # [rad/s]
        

    def contactSensorCallback(self,msg_arr): ## Callback to indicate which pads have collided with ceiling

        for msg in msg_arr.states: ## ContactsState message includes an array of ContactState messages
            # If pad collision or body collision detected then mark True
            if msg.collision1_name  ==  f"{self.modelName}::pad_1::collision" and self.pad_contacts[0] == False:
                self.pad_contacts[0] = True

                if self.logging_flag:
                    # self.append_csv(error_str="Leg_1 Contact") # Append data at instant when ceiling impact detected
                    pass
                
            elif msg.collision1_name == f"{self.modelName}::pad_2::collision" and self.pad_contacts[1] == False:
                self.pad_contacts[1] = True

                if self.logging_flag:
                    # self.append_csv(error_str="Leg_2 Contact")
                    pass

            elif msg.collision1_name == f"{self.modelName}::pad_3::collision" and self.pad_contacts[2] == False:
                self.pad_contacts[2] = True

                if self.logging_flag:
                    # self.append_csv(error_str="Leg_3 Contact")
                    pass

            elif msg.collision1_name == f"{self.modelName}::pad_4::collision" and self.pad_contacts[3] == False:
                self.pad_contacts[3] = True

                if self.logging_flag:
                    # self.append_csv(error_str="Leg_4 Contact")
                    pass

            elif msg.collision1_name == f"{self.modelName}::crazyflie_body::body_collision" and self.body_contact == False:
                self.body_contact = True

        if (any(self.pad_contacts) or self.body_contact) and self.impact_flag == False: # If any pad contacts are True then update impact flag
            self.impact_flag = True
            
            # Save state data at time of impact
            self.t_impact = self.t
            self.state_impact = self.state_current
            self.FM_impact = self.FM

    def ceiling_ftsensorCallback(self,ft_msg):
        # Keeps record of max impact force for each run
        # The value is reset in the training script at each run start

        if np.abs(ft_msg.wrench.force.z) > np.abs(self.ceiling_ft_z):
            self.ceiling_ft_z = ft_msg.wrench.force.z           # Z-Impact force from ceiling Force-Torque sensor
            self.ceiling_ft_z = np.round(self.ceiling_ft_z,3)   # Rounded for data logging

        if np.abs(ft_msg.wrench.force.x) > np.abs(self.ceiling_ft_x):
            self.ceiling_ft_x = ft_msg.wrench.force.x           # X-Impact force from ceiling Force-Torque sensor
            self.ceiling_ft_x = np.round(self.ceiling_ft_x,3)   # Rounded for data logging

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
        time.sleep(1)
        self.launch_sim()

        self.reset_pos()
        rospy.wait_for_message('/global_state',Odometry) # Wait for global state message before resuming training

    def close_sim(self):
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)
        

    def close_dashboard(self):
        os.killpg(self.dashboard_p.pid, signal.SIGTERM)

    def __del__(self):
        self.isRunning = False
        os.killpg(self.gazebo_p.pid, signal.SIGTERM)
        os.killpg(self.dashboard_p.pid, signal.SIGTERM)
        os.killpg(self.controller_p.pid, signal.SIGTERM)
     
    def getTime(self):
        return self.t

    def launch_sim(self):
        
        print("[STARTING] Starting Gazebo Process...")
        self.gazebo_p = subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory  --geometry 70x42+3154+154 -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility/launch_gazebo.bash", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
        
        


    def launch_dashboard(self):
        print("[STARTING] Starting Dashboard...")
        self.dashboard_p = subprocess.Popen(
            "gnome-terminal -- roslaunch dashboard_gui_pkg dashboard.launch",
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
        self.step('tumble',ctrl_flag=0)
        self.step('sticky',ctrl_flag=0)
        self.step('home')
        
        

        ## RESET POSITION AND VELOCITY
        state_msg = ModelState()
        state_msg.model_name = self.modelName
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0.0
        state_msg.pose.position.z = 0.544

        state_msg.pose.orientation.w = 1.0
        state_msg.pose.orientation.x = 0.0
        state_msg.pose.orientation.y = 0.0
        state_msg.pose.orientation.z = 0.0
        
        state_msg.twist.linear.x = 0.0
        state_msg.twist.linear.y = 0.0
        state_msg.twist.linear.z = 0.0

        rospy.wait_for_service('/gazebo/set_model_state')
        set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state_srv(state_msg)

        ## WAIT FOR CONTROLLER TO UPDATE STATE x2 BEFORE TURNING ON TUMBLE DETECTION
        rospy.wait_for_message('/global_state',Odometry)
        rospy.wait_for_message('/global_state',Odometry)
        self.step('tumble',ctrl_flag=1)

        # time.sleep(0.1) # Give it time for controller to receive new states
        # rospy.wait_for_service('/gazebo/get_link_state')

        ## RESET TO HOME
        

    def step(self,action,ctrl_vals=[0,0,0],ctrl_flag=1):
        cmd_msg = RLCmd()

        cmd_dict = {'home':0,
                    'pos':1,
                    'vel':2,
                    'att':3,
                    'tumble':4,
                    'stop':5,
                    'gains':6,
                    'moment':7,
                    'policy':8,
                    'sticky':11}
        

        cmd_msg.cmd_type = cmd_dict[action]
        cmd_msg.cmd_vals.x = ctrl_vals[0]
        cmd_msg.cmd_vals.y = ctrl_vals[1]
        cmd_msg.cmd_vals.z = ctrl_vals[2]
        cmd_msg.cmd_flag = ctrl_flag
        
        self.Cmd_Publisher.publish(cmd_msg) # For some reason it doesn't always publish
        self.Cmd_Publisher.publish(cmd_msg) # So I'm sending it twice 
        
        

    # ============================
    ##      Data Logging 
    # ============================
    
    def create_csv(self,filepath):

        if self.logging_flag:
        
            with open(filepath,mode='w') as state_file:
                state_writer = csv.writer(state_file,delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    'k_ep','k_run',
                    'alpha_mu','alpha_sig',
                    'mu','sigma', 'policy',
                    't','x','y','z',
                    'qw','qx','qy','qz',
                    'vx','vy','vz',
                    'wx','wy','wz',
                    'reward','flip_flag','impact_flag','n_rollouts',
                    'RREV','OF_x','OF_y',
                    'MS1','MS2','MS3','MS4',
                    'F_thrust','Mx','My','Mz',
                    'Error'])# Place holders

    def append_csv(self,error_str= ""):

        if self.logging_flag:
            with open(self.filepath, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    self.k_ep,self.k_run,
                    "","", # alpha_mu,alpha_sig
                    "","","", # mu,sigma,policy
                    self.t,self.position[0],self.position[1],self.position[2], # t,x,y,z
                    self.orientation_q[0],self.orientation_q[1],self.orientation_q[2],self.orientation_q[3], # qw,qx,qy,qz
                    self.velocity[0],self.velocity[1],self.velocity[2], # vx,vy,vz
                    self.omega[0],self.omega[1],self.omega[2], # wx,wy,wz
                    "",self.flip_flag,self.impact_flag,self.n_rollouts, # reward, flip_triggered, impact_flag, n_rollout
                    self.RREV,self.OF_x,self.OF_y, # RREV, OF_x, OF_y
                    self.MS[0],self.MS[1],self.MS[2],self.MS[3], # MS1, MS2, MS3, MS4
                    self.FM[0],self.FM[1],self.FM[2],self.FM[3], # F_thrust,Mx,My,Mz 
                    error_str]) # Error

    

    def append_flip(self):
        if self.logging_flag:
    
            with open(self.filepath,mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    self.k_ep,self.k_run,
                    "","", # alpha_mu,alpha_sig
                    "","","", # mu,sigma,policy
                    self.t_flip,self.pos_flip[0],self.pos_flip[1],self.pos_flip[2],    # t,x,y,z
                    self.quat_flip[0],self.quat_flip[1],self.quat_flip[2],self.quat_flip[3],    # qw,qx,qy,qz
                    self.vel_flip[0],self.vel_flip[1],self.vel_flip[2],    # vx_d,vy_d,vz_d
                    self.omega_flip[0],self.omega_flip[1],self.omega_flip[2],  # wx,wy,wz
                    "","","","", # reward, body_impact, num leg contacts, impact force
                    self.RREV_tr,"",self.OF_y_tr, # RREV, OF_x, OF_y
                    "","","","", # MS1, MS2, MS3, MS4
                    self.FM_flip[0],self.FM_flip[1],self.FM_flip[2],self.FM_flip[3], # F_thrust,Mx,My,Mz
                    "Flip Data"]) # Error
    
    def append_impact(self):
        if self.logging_flag:
    
            with open(self.filepath,mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    self.k_ep,self.k_run,
                    "","", # alpha_mu,alpha_sig
                    "","","", # mu,sigma,policy
                    self.t_impact,self.state_impact[0],self.state_impact[1],self.state_impact[2],    # t,x,y,z
                    self.state_impact[3],self.state_impact[4],self.state_impact[5],self.state_impact[6],    # qx,qy,qz,qw
                    self.state_impact[7],self.state_impact[8],self.state_impact[9],    # vx_d,vy_d,vz_d
                    self.state_impact[10],self.state_impact[11],self.state_impact[12],  # wx,wy,wz
                    "",self.body_contact,sum(self.pad_contacts),"",  # "", "", body_impact flag, num leg contacts, ""
                    self.ceiling_ft_z,self.ceiling_ft_x,"",             # Max impact force [z], =Max impact force [x], ""
                    "","","","", # MS1, MS2, MS3, MS4
                    self.FM_impact[0],self.FM_impact[1],self.FM_impact[2],self.FM_impact[3], # F_thrust,Mx,My,Mz
                    "Impact Data"]) # Error

    def append_IC(self):
        if self.logging_flag:
    
            with open(self.filepath,mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    self.k_ep,self.k_run,
                    np.round(self.alpha_mu,2),np.round(self.alpha_sigma,2), # alpha_mu,alpha_sig
                    np.round(self.mu,2),np.round(self.sigma,2),np.round(self.policy,2), # mu,sigma,policy
                    "","","","",    # t,x,y,z
                    "", "", "", "", # qx,qy,qz,qw
                    np.round(self.vel_d[0],2),np.round(self.vel_d[1],2),np.round(self.vel_d[2],2), # vx_d,vy_d,vz_d
                    "","","", # wx,wy,wz
                    np.round(self.reward,2),np.round(self.reward_inputs,3),"","", # reward, 
                    "","","",           # RREV, OF_x, OF_y
                    "","","","",        # MS1, MS2, MS3, MS4
                    "","","","",        # F_thrust,Mx,My,Mz
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
        self.launch_sim()

    # ============================
    ##  Control Playground Func. 
    # ============================

    def cmd_send(self):
        while True:
            # Converts input number into action name
            cmd_dict = {0:'home',1:'pos',2:'vel',3:'att',4:'tumble',5:'stop',6:'gains'}
            try:
                val = float(input("\nCmd Type (0:home,1:pos,2:vel,3:att,4:omega,5:stop,6:gains): "))
            except:
                continue
            action = cmd_dict[val]

            if action=='home' or action == 'stop': # Execute home or stop action
                ctrl_vals = [0,0,0]
                ctrl_flag = 1
                self.step(action,ctrl_vals,ctrl_flag)

            elif action=='gains': # Execture Gain changer
                
                vals = input("\nControl Gains (kp_x,kd_x,kp_R,kd_R): ") # Take in comma-seperated values and convert into list
                vals = [float(i) for i in vals.split(',')]
                ctrl_vals = vals[0:3]
                ctrl_flag = vals[3]

                self.step(action,ctrl_vals,ctrl_flag)
                
            elif action == 'tumble': # Execture Angular rate action

                ctrl_vals = input("\nControl Vals (x,y,z): ")
                ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
                ctrl_flag = 1.0

                self.step('tumble',ctrl_vals,ctrl_flag)


            else:
                ctrl_vals = input("\nControl Vals (x,y,z): ")
                ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
                ctrl_flag = float(input("\nController On/Off (1,0): "))
                self.step(action,ctrl_vals,ctrl_flag)