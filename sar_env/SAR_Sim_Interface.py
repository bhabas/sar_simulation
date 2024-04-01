#!/usr/bin/env python3
import numpy as np

import os
import time
import subprocess
from threading import Thread,Event
import rospy
import yaml
from sar_env import SAR_Base_Interface


from std_srvs.srv import Empty,EmptyRequest
from rosgraph_msgs.msg import Clock
from sar_msgs.srv import Inertia_Params,Inertia_ParamsRequest
from sar_msgs.srv import CTRL_Get_Obs,CTRL_Get_ObsRequest
from sar_msgs.srv import World_Step,World_StepRequest

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from sar_msgs.msg import SAR_StateData,SAR_TriggerData,SAR_ImpactData,SAR_MiscData



YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
CYAN = '\033[96m'
RESET = '\033[0m'

class SAR_Sim_Interface(SAR_Base_Interface):

    def __init__(self,GZ_Timeout=False):
        SAR_Base_Interface.__init__(self)
        self.loadSimParams()

        

        self.GZ_Sim_process = None
        self.SAR_DC_process = None
        self.SAR_Ctrl_process = None

        self.GZ_ping_ok = False
        self.SAR_DC_ping_ok = False
        self.SAR_Ctrl_ping_ok = False
        self.NaN_check_ok = False
        self.Sim_Status = "Initializing"
        self.Sim_Restarting = False

        

        ## START SIMULATION
        self._kill_Sim()
        self._restart_Sim()
        self._start_monitoring_subprocesses()
        # self.Sim_Status = "Running"
        self._wait_for_sim_running()

        # ## WAIT TILL TOPIC DATA STARTS COMING IN

        print("[INITIATING] Gazebo simulation started")

    def loadSimParams(self):

        ## LOAD BASE PARAMETERS
        param_path = f"{self.BASE_PATH}/sar_config/Sim_Settings.yaml"
                    
        with open(param_path, 'r') as file:
            loaded_parameters = yaml.safe_load(file)

        # Load parameters into the ROS Parameter Server
        for param_name, param_value in loaded_parameters.items():
            rospy.set_param(param_name, param_value)


    # =========================
    ##     ENV INTERACTION
    # =========================
    

    def _iterStep(self,n_steps:int = 10):
        """Update simulation by n timesteps

        Args:
            n_steps (int, optional): Number of timesteps to step through. Defaults to 10.
        """        

        self.callService('/ENV/World_Step',World_StepRequest(n_steps),World_Step)


    def _getTick(self):

        resp = self.callService('/CTRL/Get_Obs',CTRL_Get_ObsRequest(),CTRL_Get_Obs)

        return resp.Tick
    
    def _getObs(self):

        resp = self.callService('/CTRL/Get_Obs',CTRL_Get_ObsRequest(),CTRL_Get_Obs)

        Tau_CR = resp.Tau_CR
        Theta_x = resp.Theta_x
        D_perp_CR = resp.D_perp_CR
        Plane_Angle_rad = np.radians(resp.Plane_Angle_deg)

        obs_list = [Tau_CR,Theta_x,D_perp_CR,self.Plane_Angle_rad]

        Tau_CR_scaled = self.scaleValue(Tau_CR,original_range=[-5,5],target_range=[-1,1])
        Theta_x_scaled = self.scaleValue(Theta_x,original_range=[-20,20],target_range=[-1,1])
        D_perp_CR_scaled = self.scaleValue(D_perp_CR,original_range=[-0.5,2.0],target_range=[-1,1])
        Plane_Angle_scaled = self.scaleValue(self.Plane_Angle_deg,original_range=[0,180],target_range=[-1,1])

        scaled_obs_list = [Tau_CR_scaled,Theta_x_scaled,D_perp_CR_scaled,Plane_Angle_scaled]

        ## OBSERVATION VECTOR
        obs = np.array(scaled_obs_list,dtype=np.float32)

        return obs

    def sleep(self,time_s):
        """
        Sleep in terms of Gazebo sim seconds not real time seconds
        """

        t_start = self.t
        while self.t - t_start <= time_s:

            ## NEGATIVE TIME DELTA
            if self.t < t_start:
                self.Done = True
                return False

        return True
        
    def Sim_VelTraj(self,pos,vel): 
        """
        Args:
            pos (list): Launch position [m]   | [x,y,z]
            vel (list): Launch velocity [m/s] | [Vx,Vy,Vz]
        """        

        ## CREATE SERVICE MESSAGE
        state_srv = ModelState()
        state_srv.model_name = self.SAR_Config

        ## INPUT POSITION AND ORIENTATION
        state_srv.pose.position.x = pos[0]
        state_srv.pose.position.y = pos[1]
        state_srv.pose.position.z = pos[2]

        ## PUBLISH MODEL STATE SERVICE REQUEST
        self.pausePhysics(pause_flag=True)
        self.callService('/gazebo/set_model_state',state_srv,SetModelState)
        self.sendCmd('Pos',cmd_vals=pos,cmd_flag=1)
        self._iterStep(100)
        

        ## CREATE SERVICE MESSAGE
        state_srv = ModelState()
        state_srv.model_name = self.SAR_Config

        ## INPUT POSITION AND ORIENTATION
        state_srv.pose.position.x = pos[0]
        state_srv.pose.position.y = pos[1]
        state_srv.pose.position.z = pos[2]

        ## INPUT LINEAR AND ANGULAR VELOCITY
        state_srv.twist.linear.x = vel[0]
        state_srv.twist.linear.y = vel[1]
        state_srv.twist.linear.z = vel[2]

        ## PUBLISH MODEL STATE SERVICE REQUEST
        self.pausePhysics(pause_flag=True)
        self.callService('/gazebo/set_model_state',state_srv,SetModelState)
        self._iterStep(2)


        ## SET DESIRED VEL IN CONTROLLER
        self.sendCmd('GZ_Const_Vel_Traj',cmd_vals=[np.nan,vel[0],0],cmd_flag=0)
        self._iterStep(2)
        self.sendCmd('GZ_Const_Vel_Traj',cmd_vals=[np.nan,vel[1],0],cmd_flag=1)
        self._iterStep(2)
        self.sendCmd('GZ_Const_Vel_Traj',cmd_vals=[np.nan,vel[2],0],cmd_flag=2)
        self._iterStep(2)
        self.sendCmd('Activate_traj',cmd_vals=[1,1,1])

        ## ROUND OUT TO 10 ITER STEPS (0.01s) TO MATCH 100Hz CONTROLLER 
        self._iterStep(2)

    def resetPose(self,z_0=0.5):           

        self.sendCmd('GZ_StickyPads',cmd_flag=0)

        self.sendCmd('Tumble_Detect',cmd_flag=0)
        self.sendCmd('Ctrl_Reset')
        self._setModelState(pos=[0,0,z_0])
        self._iterStep(10)

        self.sendCmd('Tumble_Detect',cmd_flag=1)
        self.sendCmd('Ctrl_Reset')
        self._setModelState(pos=[0,0,z_0])
        self._iterStep(100) # Give time for drone to settle

        self.sendCmd('GZ_StickyPads',cmd_flag=1)

    def _setModelState(self,pos=[0,0,0.5],quat=[0,0,0,1],vel=[0,0,0],ang_vel=[0,0,0]):

        ## RESET POSITION AND VELOCITY
        state_srv = ModelState()
        state_srv.model_name = self.SAR_Config
        state_srv.pose.position.x = pos[0]
        state_srv.pose.position.y = pos[1]
        state_srv.pose.position.z = pos[2]

        state_srv.pose.orientation.x = quat[0]
        state_srv.pose.orientation.y = quat[1]
        state_srv.pose.orientation.z = quat[2]
        state_srv.pose.orientation.w = quat[3]
        
        state_srv.twist.linear.x = vel[0]
        state_srv.twist.linear.y = vel[1]
        state_srv.twist.linear.z = vel[2]

        state_srv.twist.angular.x = ang_vel[0]
        state_srv.twist.angular.y = ang_vel[1]
        state_srv.twist.angular.z = ang_vel[2]

        self.callService('/gazebo/set_model_state',state_srv,SetModelState)

    def _setModelInertia(self,Mass=0,Inertia=[0,0,0]):

        ## CREATE SERVICE REQUEST MSG
        srv = Inertia_ParamsRequest() 
        srv.Mass = Mass
        srv.Inertia.x = Inertia[0]
        srv.Inertia.y = Inertia[1]
        srv.Inertia.z = Inertia[2]

        ## SEND LOGGING REQUEST VIA SERVICE
        self.callService("/SAR_Internal/Inertia_Update",srv,Inertia_Params)

    def scaleValue(self,x, original_range=(-1, 1), target_range=(-1, 1)):

        original_min, original_max = original_range
        target_min, target_max = target_range

        # Scale x to [0, 1] in original range
        x_scaled = (x - original_min) / (original_max - original_min)

        # Scale [0, 1] to target range
        x_target = x_scaled * (target_max - target_min) + target_min
        return x_target
    
    # ============================
    ##      Command Handlers 
    # ============================
        
    ## ========== GAZEBO FUNCTIONS ==========
    def handle_Load_Params(self):

        print("Reset ROS Parameters\n")
        self.loadBaseParams()
        self.loadSimParams()
        self.sendCmd("Load_Params")
        
    def handle_GZ_StickyPads(self):
        cmd_flag = self.userInput("Turn sticky pads On/Off (1,0): ",int)
        self.sendCmd("GZ_StickyPads",cmd_flag=cmd_flag)

    def handle_GZ_Pose_Reset(self):
        print("Reset Pos/Vel -- Sticky off -- Controller Reset\n")
        self.resetPose()
        self.pausePhysics(pause_flag=False)

    def handle_GZ_Global_Vel_traj(self):

        ## GET GLOBAL VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        ## CALC GLOBAL VELOCITIES
        Vx = V_mag*np.cos(np.radians(V_angle))
        Vy = 0
        Vz = V_mag*np.sin(np.radians(V_angle))
        V_B_O = [Vx,Vy,Vz]

        self.Sim_VelTraj(self.r_B_O,V_B_O)
        self.pausePhysics(False)

    def handle_GZ_Rel_Vel_traj(self):

        ## GET RELATIVE VEL CONDITIONS 
        V_mag,V_angle = self.userInput("Flight Velocity (V_mag,V_angle):",float)

        ## CALC RELATIVE VELOCITIES
        V_tx = V_mag*np.cos(np.radians(V_angle))
        V_ty = 0
        V_perp = V_mag*np.sin(np.radians(V_angle))

        ## CALCULATE GLOBAL VELOCITIES
        V_B_O = self.R_PW(np.array([V_tx,V_ty,V_perp]),self.Plane_Angle_rad)

        self.Sim_VelTraj(self.r_B_O,V_B_O)
        self.pausePhysics(False)

    
    # ================================
    ##     SIM MONITORING/LAUNCH
    # ================================

    def _launch_GZ_Sim(self):
        cmd = "gnome-terminal --disable-factory  --geometry 85x46+1050+0 -- rosrun sar_launch launch_gazebo.bash"
        self.GZ_Sim_process = subprocess.Popen(cmd, shell=True)

    def _launch_SAR_DC(self):
        cmd = "gnome-terminal --disable-factory  --geometry 85x46+1050+0 -- rosrun sar_data_converter SAR_DataConverter"
        self.SAR_DC_process = subprocess.Popen(cmd, shell=True)

    def _launch_Controller(self):
        cmd = "gnome-terminal --disable-factory  --geometry 85x46+1050+0 -- rosrun sar_control SAR_Controller"
        self.SAR_Ctrl_process = subprocess.Popen(cmd, shell=True)

    


    def _start_monitoring_subprocesses(self):
        monitor_thread = Thread(target=self._monitor_subprocesses)
        monitor_thread.daemon = True
        monitor_thread.start()


    def _monitor_subprocesses(self):

        while True:

            self.GZ_ping_ok = self._ping_service("/gazebo/get_loggers",timeout=5)
            self.SAR_DC_ping_ok = self._ping_service("/SAR_DataConverter_Node/get_loggers",timeout=5)
            self.SAR_Ctrl_ping_ok = self._ping_service("/SAR_Controller_Node/get_loggers",timeout=5)
            self.NaN_check_ok = not np.isnan(self.r_B_O[0])

            if not (self.GZ_ping_ok and self.SAR_DC_ping_ok and self.SAR_Ctrl_ping_ok):
                self.Sim_Status = "Restarting"
                error_msg = YELLOW + "[WARNING] One or more subprocesses not responding. Restarting all subprocesses..." + RESET
                self.Done = True
                print(error_msg)
                self._restart_Sim()

            elif not (self.NaN_check_ok):
                self.Sim_Status = "NaN Detected"
                error_msg = YELLOW + "[WARNING] NaN value detected. Restarting all subprocesses..." + RESET
                self.Done = True
                print(error_msg)
                self._restart_Sim()
                
            else:
                self.Sim_Status = "Running"
                error_msg = ""

            time.sleep(0.5)

    def _wait_for_sim_running(self,timeout=600):

        ## BLOCKING WAIT FOR SIM TO BE RUNNING
        start_time = time.time()
        while not (self.Sim_Status == "Running"):

            elapsed_time = time.time() - start_time
            self.Sim_Restarting = True

            if elapsed_time > timeout:
                raise TimeoutError("Timeout reached while waiting for simulation to fully launch.")
            else:
                print(YELLOW + "Waiting for simulation to fully launch..." + RESET)
                time.sleep(1.0)
        
        if self.Sim_Restarting == True:
            print(GREEN + "Simulation has fully launched." + RESET)
            self.Sim_Restarting = False


    def _ping_service(self, service_name,timeout=5,silence_errors=False):
        cmd = f"rosservice call {service_name}"
        stderr_option = subprocess.DEVNULL if silence_errors else None

        try:
            result = subprocess.run(cmd, shell=True, timeout=timeout, check=True, stdout=subprocess.PIPE, stderr=stderr_option)
            return result.returncode == 0
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            return False
        
    def _kill_Sim(self):

        ## KILL ALL POTENTIAL NODE/SUBPROCESSES
        os.system("killall -9 gzserver gzclient")
        os.system("rosnode kill /gazebo /gazebo_gui")
        time.sleep(1.0)
        os.system("rosnode kill /SAR_Controller_Node")
        time.sleep(1.0)
        os.system("rosnode kill /SAR_DataConverter_Node")
        time.sleep(1.0)


    def _restart_Sim(self):

        ## LAUNCH GAZEBO
        self._launch_GZ_Sim()

        if rospy.get_param(f"/SIM_SETTINGS/GUI_Flag") == True:
            self._wait_for_node(node_name="gazebo_gui",timeout=60,interval=2)
        else:
            self._wait_for_node(node_name="gazebo",timeout=60,interval=2)


        ## LAUNCH CONTROLLER
        self._launch_Controller()
        self._wait_for_node(node_name="SAR_Controller_Node",timeout=10,interval=0.5)

        ## LAUNCH SAR_DC
        self._launch_SAR_DC()
        self._wait_for_node(node_name="SAR_DataConverter_Node",timeout=10,interval=0.5)

    
    def _wait_for_node(self,node_name,timeout=None,interval=1.0):

        start_time = time.time()

        while not self._ping_service(f"/{node_name}/get_loggers",silence_errors=True):

            if timeout is not None and time.time() - start_time > timeout:
                print(f"Timeout reached while waiting for {node_name} to launch.")
                return False
        
            print(f"Waiting for {node_name} to fully launch...")
            time.sleep(interval)

        print(f"{node_name} has fully launched.")
        return True
    
    def callService(self,srv_addr,srv_msg,srv_type,num_retries=5,call_timeout=5):
            
        ## CALL SERVICE AND RETURN RESPONSE
        service_proxy = rospy.ServiceProxy(srv_addr, srv_type)

        def _service_call_thread(service_proxy,srv_msg):
            try:
                self.response = service_proxy(srv_msg)
            except (rospy.ServiceException,rospy.exceptions.ROSException) as e:
                rospy.logerr(f"Service call failed with exception: {e}")

        for retry in range(num_retries):

            self.response = None
            thread = Thread(target=_service_call_thread, args=(service_proxy,srv_msg,))
            thread.start()

            # Wait for the specified timeout
            thread.join(timeout=call_timeout)

            if thread.is_alive():
                # Thread is still alive, meaning it didn't finish within the timeout
                rospy.logwarn(f"Service call timeout after {call_timeout} seconds.")
                # Here you may decide to stop trying or take other actions
                # Note: the thread will still be running in the background
            elif self.response is not None:
                # Service call was successful
                return self.response
            
            rospy.logwarn(f"Retrying service call ({retry+1}/{num_retries}).")

        # If the service call has failed all retries
        rospy.logerr(f"Service '{srv_addr}' call failed after {num_retries} attempts.")
        self.Done = True
        self._kill_Sim()
        self._wait_for_sim_running()
        return None

    def pausePhysics(self,pause_flag=True):

        if pause_flag == True:
            self.callService("/gazebo/pause_physics",EmptyRequest(),Empty)

        else:
            self.callService("/gazebo/unpause_physics",EmptyRequest(),Empty)

        

 

if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = SAR_Sim_Interface()

    rospy.spin()


