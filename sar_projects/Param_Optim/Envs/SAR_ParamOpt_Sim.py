#!/usr/bin/env python3
import numpy as np
import time

from sar_env import SAR_Sim_Interface


class SAR_ParamOpt_Sim(SAR_Sim_Interface):

    def __init__(self,GZ_Timeout=False):
        SAR_Sim_Interface.__init__(self)        

        self.Env_Name = "SAR_ParamOptim_Env"
        self.k_ep = 0
    
        self.D_min = np.inf
        self.Tau_trg = np.inf
        self.done = False

        self.GZ_Timeout = GZ_Timeout


    def ParamOptim_reset(self):

        ## RESET REWARD CALC VALUES
        self.done = False
        self.D_min = np.inf  # Reset max from ceiling [m]
        self.Tau_trg = np.inf
        

        self.SendCmd('GZ_StickyPads',cmd_flag=0)

        self.SendCmd('Tumble',cmd_flag=0)
        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.sleep(0.01)

        self.SendCmd('Tumble',cmd_flag=1)
        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.sleep(1.0) # Give time for drone to settle

        self.SendCmd('GZ_StickyPads',cmd_flag=1)


        # ## DOMAIN RANDOMIZATION (UPDATE INERTIA VALUES)
        # self.Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Iyy") + np.random.normal(0,1.5e-6)
        # self.mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Mass") + np.random.normal(0,0.0005)
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


        vz = vel*np.sin(np.deg2rad(phi))
        vx = vel*np.cos(np.deg2rad(phi))

        # tau_0 = 0.5
        # z_0 = 2.10 - tau_0*vz
        z_0 = 1.0

        self.Vel_Launch([0,0,z_0],[vx,0,vz])
        self.pause_physics(False)
        self.sleep(0.05)
        self.SendCmd("Policy",cmd_vals=[Tau,My,0.0],cmd_flag=1)


        

        while not self.done: 

            t_now = self.getTime()

            ## RECORD LOWEST D_perp VALUE
            if self.D_perp < self.D_min:
                self.D_min = self.D_perp 

            ## START FLIP AND IMPACT TERMINATION TIMERS
            if (self.flip_flag == True and onceFlag_flip == False):
                start_time_pitch = t_now    # Starts countdown for when to reset run
                onceFlag_flip = True        # Turns on to make sure this only runs once per rollout

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
                self.restart_Sim()
                self.done = True




        reward = self._CalcReward()

        return None,reward,self.done,None
    
    def _CalcReward(self):

        ## DISTANCE REWARD 
        R_dist = np.clip(1/np.abs(self.D_min + 1e-3),0,15)/15
        
        ## TAU TRIGGER REWARD
        R_tau = np.clip(1/np.abs(self.Tau_trg - 0.2),0,15)/15


        ## IMPACT ANGLE REWARD # (Derivation: Research_Notes_Book_3.pdf (6/21/23))
        Beta_d = 150 # Desired impact angle [= 180 deg - gamma [deg]] TODO: Change to live value?
        Beta_rel = -180 + self.Rot_Sum_impact + self.Plane_Angle

        if Beta_rel < -180:
            R_angle = 0
        elif -180 <= Beta_rel < 180:
            R_angle = 0.5*(-np.cos(np.deg2rad(Beta_rel*180/Beta_d))+1)
        elif Beta_rel > 180:
            R_angle = 0

        ## PAD CONTACT REWARD
        if self.pad_connections >= 3: 
            if self.BodyContact_flag == False:
                R_legs = 1.0
            else:
                R_legs = 0.3

        elif self.pad_connections == 2: 
            if self.BodyContact_flag == False:
                R_legs = 0.6
            else:
                R_legs = 0.1
                
        else:
            R_legs = 0.0

        self.reward_vals = [R_dist,R_tau,R_angle,R_legs,0]
        self.reward = 0.05*R_dist + 0.1*R_tau + 0.2*R_angle + 0.65*R_legs

        return self.reward

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

        self.reward_vals = [R1,R2,R3,0,0]

        return R1 + R2 + R3



if __name__ == "__main__":

    env = SAR_ParamOpt_Sim(GZ_Timeout=False)

    for ii in range(1000):
        Tau_trg = 0.23
        My = 8
        Vel_d = 2.5
        Phi_d = 30
        env.ParamOptim_reset()
        obs,reward,done,info = env.ParamOptim_Flight(Tau_trg,My,Vel_d,Phi_d)
        print(f"Ep: {ii} \t Reward: {reward:.02f}")
