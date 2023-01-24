#!/usr/bin/env python3
import numpy as np
import time
import rospy


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)

from crazyflie_env.Core_Envs.CrazyflieEnv_Sim import CrazyflieEnv_Sim


class CrazyflieEnv_ParamOpt(CrazyflieEnv_Sim):
    metadata = {'render.modes': ['human']}
    def __init__(self,GZ_Timeout=False):
        CrazyflieEnv_Sim.__init__(self)        

        self.env_name = "CF_PO"
        self.k_ep = 0
    
        self.D_min = 50.0
        self.done = False

        self.GZ_Timeout = GZ_Timeout


    def ParamOptim_reset(self):

        ## RESET REWARD CALC VALUES
        self.done = False
        self.D_min = 50.0  # Reset max from ceiling [m]

        ## DISABLE STICKY LEGS (ALSO BREAKS CURRENT CONNECTION JOINTS)
        self.SendCmd('Tumble',cmd_flag=0)
        self.SendCmd('StickyPads',cmd_flag=0)

        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.SendCmd('Ctrl_Reset')
        self.reset_pos()

        self.SendCmd('Tumble',cmd_flag=1)
        self.sleep(0.25)

        # ## DOMAIN RANDOMIZATION (UPDATE INERTIA VALUES)
        # self.Iyy = rospy.get_param(f"/CF_Type/{self.CF_Type}/Config/{self.CF_Config}/Iyy") + np.random.normal(0,1.5e-6)
        # self.mass = rospy.get_param(f"/CF_Type/{self.CF_Type}/Config/{self.CF_Config}/Mass") + np.random.normal(0,0.0005)
        # self.updateInertia()

        

        obs = None
        return obs

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

        # tau_0 = 0.5
        # z_0 = 2.10 - tau_0*vz
        z_0 = 1.0

        self.Vel_Launch([0,0,z_0],[vx,0,vz])
        self.gazebo_unpause_physics()
        self.sleep(0.05)
        self.SendCmd("Policy",cmd_vals=[Tau,My,0.0],cmd_flag=1)


        

        while not self.done: 

            t_now = self.getTime()

            ## RECORD LOWEST D_perp VALUE
            if self.D_perp < self.D_min:
                self.D_min = self.D_perp 

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
            elif self.vel[2] <= -0.5 and self.pos[2] <= 1.5: 
                self.error_str = "Rollout Completed: Falling Drone"
                self.done = True
                # print(self.error_str)



            if (time.time() - start_time_ep) > 15.0 and self.GZ_Timeout == True:
                print('\033[93m' + "[WARNING] Real Time Exceeded" + '\x1b[0m')
                self.Restart()
                self.done = True


            if any(np.isnan(self.vel)): 
                self.error_str = "Rollout Completed: NaN in State Vector"

                print('\033[93m' + "[WARNING] NaN in State Vector" + '\x1b[0m')
                self.Restart([True,True,True])
                print('\033[93m' + "[WARNING] Resuming Flight" + '\x1b[0m')
                self.done = True
                # print(self.error_str)

        reward = self.CalcReward()

        return None,reward,self.done,None

    def CalcReward(self):

        ## DISTANCE REWARD 
        R1 = np.clip(1/np.abs(self.D_min+1e-3),0,10)/10
        R1 *= 0.1

        ## IMPACT ANGLE REWARD
        R2 = np.clip(np.abs(self.eul_impact[1])/90,0,1)
        R2 *= 0.3

        ## PAD CONTACT REWARD
        if self.pad_connections >= 3: 
            if self.BodyContact_flag == False:
                R3 = 0.6
            else:
                R3 = 0.3
        elif self.pad_connections == 2: 
            if self.BodyContact_flag == False:
                R3 = 0.2
            else:
                R3 = 0.1
        else:
            R3 = 0.0

        return R1 + R2 + R3



if __name__ == "__main__":

    env = CrazyflieEnv_ParamOpt(GZ_Timeout=False)

    for ii in range(1000):
        Tau_tr = 0.2
        My = 8
        Vel_d = 2.5
        Phi_d = 60
        env.ParamOptim_reset()
        obs,reward,done,info = env.ParamOptim_Flight(Tau_tr,My,Vel_d,Phi_d)
        print(f"Ep: {ii} \t Reward: {reward:.02f}")
