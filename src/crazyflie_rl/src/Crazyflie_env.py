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
        self.state_current = np.zeros(14)


        self.isRunning = True
        self.username = getpass.getuser()
        self.loggingPath =  f"/home/{self.username}/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log"
        self.logging_flag = False
        self.filepath = ""
        self.runComplete_flag = False

        self.h_ceiling = 3.00

        ## INIT NAME OF MODEL BEING USED
        self.modelName = 'crazyflie_model_Flex'

        ## INIT RL_DATA VARIABLES 
        #region 
        self.trial_name = ''
        self.agent_name = ''
        self.error_str = ''

        self.n_rollouts = 0
        self.gamma = 0
        
        self.k_run = 0
        self.k_ep = 0

        self.alpha_mu = []
        self.alpha_sigma = []
        self.mu = []
        self.sigma = []
        self.policy = []

        self.reward = 0
        self.reward_avg = 0

        self.vel_d = [0,0,0]
        self.M_d = [0,0,0]

        self.MS = [0,0,0,0]
        self.FM = [0,0,0,0]
        

        ## REWARD VALS
        self.z_max = 0.0
        self.pitch_sum = 0.0
        self.pitch_max = 0.0

        self.t_prev = 0.0

        ## FLIP VALS
        self.state_flip = np.zeros(14)
        self.FM_flip = [0,0,0,0]

        self.flip_flag = False

        ## IMPACT VALS
        self.state_impact = np.zeros(14)
        self.FM_impact = [0,0,0,0]

        self.pad_contacts = [False,False,False,False]
        self.body_contact = False
        self.ceiling_ft_z = 0.0
        self.ceiling_ft_x = 0.0
        self.impact_flag = False
        #endregion 




        
        ## INIT ROS NODE FOR ENVIRONMENT 
        rospy.init_node("crazyflie_env_node") 
        print("[STARTING] Starting Controller Process...")
        self.controller_p = subprocess.Popen( # Controller Process
            "roslaunch crazyflie_gazebo controller.launch", close_fds=True, preexec_fn=os.setsid, shell=True)
        self.launch_sim() 
    

        

        ## INIT ROS SUBSCRIBERS 
        self.state_Subscriber = rospy.Subscriber('/global_state',Odometry,self.global_stateCallback)                        # 100 Hz
        self.ctrl_Subscriber = rospy.Subscriber('/ctrl_data',CtrlData,self.ctrlCallback)                                    # 500 Hz

        self.OF_Subscriber = rospy.Subscriber('/OF_sensor',Odometry,self.OFsensor_Callback)                                 # 100 Hz
        self.contact_Subscriber = rospy.Subscriber('/ceiling_contact',ContactsState,self.contactSensorCallback)             # 750 Hz
        self.ceiling_ft_Subscriber = rospy.Subscriber('/ceiling_force_sensor',WrenchStamped,self.ceiling_ftsensorCallback)  # 750 Hz
        self.laser_Subscriber = rospy.Subscriber('/zranger2/scan',LaserScan,self.laser_sensorCallback)                      # 100 Hz
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
        rl_msg.gamma = self.gamma
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

        self.FM = np.asarray(ctrl_msg.FM) # Force/Moments [N,N*mm]
        self.FM = np.round(self.FM,3)

        
        if ctrl_msg.flip_flag == True and self.flip_flag == False: # Activtes only once per run when flip_flag is about to change

            self.flip_flag = ctrl_msg.flip_flag # Update flip_flag

            # Save state data at time of flip activation
            self.state_flip = self.state_current

            self.FM_flip = np.asarray(ctrl_msg.FM_flip) # Force/Moments [N,N*mm]
            self.FM_flip = np.round(self.FM_flip,3)
            
            self.RREV_tr = np.round(ctrl_msg.RREV_tr,3) # Recorded trigger RREV [rad/s]
            self.OF_y_tr = np.round(ctrl_msg.OF_y_tr,3) # Recorded OF_y at trigger [rad/s]



       

    def OFsensor_Callback(self,OF_msg): ## Callback to parse state data received from mock OF sensor

        ## SET VISUAL CUE SENSOR VALUES FROM TOPIC
        d = self.h_ceiling - OF_msg.pose.pose.position.z    # Distance from CF to ceiling [m]

        self.RREV = round( OF_msg.twist.twist.linear.z/d,3) # [rad/s]
        self.OF_x = round(-OF_msg.twist.twist.linear.y/d,3) # [rad/s]
        self.OF_y = round(-OF_msg.twist.twist.linear.x/d,3) # [rad/s]

    def global_stateCallback(self,gs_msg): ## Callback to parse state data received from gazebo_communication node
        
        ## SET TIME VALUE FROM TOPIC
        t_temp = gs_msg.header.stamp.secs
        ns_temp = gs_msg.header.stamp.nsecs
        self.t = np.round(t_temp+ns_temp*1e-9,3) # (seconds + nanoseconds)
        
        ## SIMPLIFY STATE VALUES FROM TOPIC
        global_pos = gs_msg.pose.pose.position
        global_quat = gs_msg.pose.pose.orientation
        global_vel = gs_msg.twist.twist.linear
        global_omega = gs_msg.twist.twist.angular
        
        if global_quat.w == 0: # If zero at startup set quat.w to one to prevent errors
            global_quat.w = 1

        ## SET STATE VALUES FROM TOPIC
        
        self.position = np.round([global_pos.x,global_pos.y,global_pos.z],3)
        self.orientation_q = np.round([global_quat.w,global_quat.x,global_quat.y,global_quat.z],3)
        self.velocity = np.round([global_vel.x,global_vel.y,global_vel.z],3)
        self.omega = np.round([global_omega.x,global_omega.y,global_omega.z],3)


        ## COMBINE INTO COMPREHENSIVE LIST
        self.state_current = np.concatenate([np.atleast_1d(self.t),self.position,self.orientation_q,self.velocity,self.omega])
        



        ## MAX Z CALC
        if self.position[2] > self.z_max:
            self.z_max = self.position[2]       # Max height achieved, used for reward calc
            self.z_max = np.round(self.z_max,3) # Rounded for data logging

        ## MAX PITCH CALC
        # Integrate omega_y over time to get full rotation estimate
        # This accounts for multiple revolutions that euler angles/quaternions can't
        self.pitch_sum = self.pitch_sum + self.omega[1]*(180/np.pi)*(self.t - self.t_prev) # [deg]
        
        if self.pitch_sum < self.pitch_max:     # Recording the most negative value
            self.pitch_max = self.pitch_sum
            self.pitch_max = np.round(self.pitch_max,3)


        self.t_prev = self.t # Save t value for next callback iteration
        

    def contactSensorCallback(self,msg_arr): ## Callback to indicate which pads have collided with ceiling

        for msg in msg_arr.states: ## ContactsState message includes an array of ContactState messages
            # If pad collision or body collision detected then mark True
            if msg.collision1_name  ==  f"{self.modelName}::pad_1::collision" and self.pad_contacts[0] == False:
                self.pad_contacts[0] = True

                if self.logging_flag:
                    self.append_csv() # Append data at instant when ceiling impact detected
                
            elif msg.collision1_name == f"{self.modelName}::pad_2::collision" and self.pad_contacts[1] == False:
                self.pad_contacts[1] = True

                if self.logging_flag:
                    self.append_csv()
            elif msg.collision1_name == f"{self.modelName}::pad_3::collision" and self.pad_contacts[2] == False:
                self.pad_contacts[2] = True

                if self.logging_flag:
                    self.append_csv()

            elif msg.collision1_name == f"{self.modelName}::pad_4::collision" and self.pad_contacts[3] == False:
                self.pad_contacts[3] = True

                if self.logging_flag:
                    self.append_csv()

            elif msg.collision1_name == f"{self.modelName}::crazyflie_body::body_collision" and self.body_contact == False:
                self.body_contact = True

        if (any(self.pad_contacts) or self.body_contact) and self.impact_flag == False: # If any pad contacts are True then update impact flag
            self.impact_flag = True
            
            # Save state data at time of impact
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

        
    # ============================
    ##    Sensors/Gazebo Topics
    # ============================

    def laser_sensorCallback(self,data): # callback function for laser subsriber
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

    def launch_IC(self,vx_d,vz_d): # Imparts desired velocity to model (should retain current position)
        
        ## SET POSE AND TWIST OF MODEL
        state_msg = ModelState()
        state_msg.model_name = self.modelName
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.6

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1

        state_msg.twist.linear.x = vx_d
        state_msg.twist.linear.y = 0
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
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.55

        state_msg.pose.orientation.w = 1
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        

        state_msg.twist.linear.x = 0
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0

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
                    'gamma','reward','flip_flag','impact_flag','n_rollouts',
                    'RREV','OF_x','OF_y',
                    'MS1','MS2','MS3','MS4',
                    'F_thrust','Mx','My','Mz',
                    'Error'])# Place holders

    def append_csv(self):

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
                    "","",self.flip_flag,self.impact_flag,self.n_rollouts, # gamma, reward, flip_triggered, impact_flag, n_rollout
                    self.RREV,self.OF_x,self.OF_y, # RREV, OF_x, OF_y
                    self.MS[0],self.MS[1],self.MS[2],self.MS[3], # MS1, MS2, MS3, MS4
                    self.FM[0],self.FM[1],self.FM[2],self.FM[3], # F_thrust,Mx,My,Mz 
                    ""]) # Error

    

    def append_flip(self):
        if self.logging_flag:
    
            with open(self.filepath,mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    self.k_ep,self.k_run,
                    "","", # alpha_mu,alpha_sig
                    "","","", # mu,sigma,policy
                    self.state_flip[0],self.state_flip[1],self.state_flip[2],self.state_flip[3],    # t,x,y,z
                    self.state_flip[4],self.state_flip[5],self.state_flip[6],self.state_flip[7],    # qx,qy,qz,qw
                    self.state_flip[8],self.state_flip[9],self.state_flip[10],    # vx_d,vy_d,vz_d
                    self.state_flip[11],self.state_flip[12],self.state_flip[13],  # wx,wy,wz
                    "","","","","", # gamma, reward, body_impact, num leg contacts, impact force
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
                    self.state_impact[0],self.state_impact[1],self.state_impact[2],self.state_impact[3],    # t,x,y,z
                    self.state_impact[4],self.state_impact[5],self.state_impact[6],self.state_impact[7],    # qx,qy,qz,qw
                    self.state_impact[8],self.state_impact[9],self.state_impact[10],    # vx_d,vy_d,vz_d
                    self.state_impact[11],self.state_impact[12],self.state_impact[13],  # wx,wy,wz
                    "","",self.body_contact,sum(self.pad_contacts),"",  # "", "", body_impact flag, num leg contacts, ""
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
                    np.round(self.gamma,2),round(self.reward,2),"","","", # gamma, reward, 
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
        self.timer_unpause = Timer(4,self.timeout_unpause)
        self.timer_unpause.start()

        ## RESET TIMER THAT RELAUNCHES SIM
        self.timer_relaunch.cancel()
        self.timer_relaunch = Timer(7,self.timeout_relaunch)
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