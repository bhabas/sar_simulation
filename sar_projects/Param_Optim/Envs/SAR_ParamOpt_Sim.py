#!/usr/bin/env python3
import numpy as np
import time

from sar_env import SAR_Sim_Interface


class SAR_ParamOpt_Sim(SAR_Sim_Interface):

    def __init__(self,GZ_Timeout=False):
        SAR_Sim_Interface.__init__(self)        

        self.Env_Name = "SAR_ParamOptim_Env"
        self.GZ_Timeout = GZ_Timeout


        ## TIME CONSTRAINTS
        self.t_trg_max = 1.0   # [s]
        self.t_impact_max = 1.0   # [s]

    
        ## INITIAL LEARNING/REWARD CONFIGS
        self.K_ep = 0
        self.Pol_Trg_Flag = False
        self.Done = False
        self.reward = 0
        self.reward_vals = np.array([0,0,0,0,0,0])
        self.reward_weights = {
            "W_Dist":0.2,
            "W_tau_cr":0.0,
            "W_LT":1.0,
            "W_GM":0.0,
            "W_Phi_rel":0.0,
            "W_Legs":2.0
        }
        self.W_max = sum(self.reward_weights.values())

        self.D_min = np.inf
        self.Tau_trg = np.inf
        self.Tau_CR_trg = np.inf

        


    def ParamOptim_reset(self):

        ######################
        #    GENERAL CONFIGS
        ######################


        ## RESET LEARNING/REWARD CONDITIONS
        self.K_ep += 1
        self.Done = False
        self.Pol_Trg_Flag = False
        self.reward = 0

        self.D_min = np.inf
        self.Tau_trg = np.inf
        self.Tau_CR_trg = np.inf

        self.Impact_Flag = False
        self.BodyContact_Flag = False
        self.Pad_Connections = 0
        self.MomentCutoff = False





        self.resetPose()

        ## RESET/UPDATE TIME CONDITIONS
        self.start_time_episode = self._get_time()
        self.start_time_trg = np.nan
        self.start_time_impact = np.nan
        

        


        # ## DOMAIN RANDOMIZATION (UPDATE INERTIA VALUES)
        # self.Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Iyy") + np.random.normal(0,1.5e-6)
        # self.Mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Mass") + np.random.normal(0,0.0005)
        # self.setModelInertia()

        

        obs = None
        return obs

    def ParamOptim_Flight(self,Tau,Rot_acc,vel,phi):

        ## RESET/UPDATE RUN CONDITIONS
        start_time_rollout = self._get_time()
        t_Rot_Action = np.nan
        start_time_impact = np.nan

        start_time_ep = time.time()

        ## RESET LOGGING CONDITIONS 
        OnceFlag_Trg = False    # Ensures Rot data recorded only once
        OnceFlag_Impact = False   # Ensures impact data recorded only once 


        vz = vel*np.sin(np.deg2rad(phi))
        vx = vel*np.cos(np.deg2rad(phi))

        # tau_0 = 0.5
        # z_0 = 2.10 - tau_0*vz
        z_0 = 0.5

        self.Vel_Launch([0,0,z_0],[vx,0,vz])
        self.pause_physics(False)
        self.sleep(0.05)
        self.sendCmd("Policy",cmd_vals=[Tau,Rot_acc,0.0],cmd_flag=1)


        

        while not self.Done: 

            t_now = self._get_time()

            ## RECORD LOWEST D_perp VALUE
            if self.D_perp < self.D_min:
                self.D_min = self.D_perp 

            ## START TRIGGER AND IMPACT TERMINATION TIMERS
            if (self.Trg_flag == True and OnceFlag_Trg == False):
                t_Rot_Action = t_now    # Starts countdown for when to reset run
                OnceFlag_Trg = True        # Turns on to make sure this only runs once per rollout

            if ((self.Impact_flag or self.BodyContact_Flag) and OnceFlag_Impact == False):
                start_time_impact = t_now
                OnceFlag_Impact = True

            # ============================
            ##    Termination Criteria 
            # ============================

            ## PITCH TIMEOUT  
            if (t_now-t_Rot_Action) > 2.25:
                self.error_str = "Rollout Completed: Pitch Timeout"
                self.Done = True
                # print(self.error_str)


            ## IMPACT TIMEOUT
            elif (t_now-start_time_impact) > 0.5:
                self.error_str = "Rollout Completed: Impact Timeout"
                self.Done = True
                # print(self.error_str)



            ## ROLLOUT TIMEOUT
            elif (t_now - start_time_rollout) > 5.0:
                self.error_str = "Rollout Completed: Time Exceeded"
                self.Done = True
                # print(self.error_str)

            ## FREE FALL TERMINATION
            elif self.vel[2] <= -0.5 and self.pos[2] <= 1.5: 
                self.error_str = "Rollout Completed: Falling Drone"
                self.Done = True
                # print(self.error_str)



            if (time.time() - start_time_ep) > 15.0 and self.GZ_Timeout == True:
                print('\033[93m' + "[WARNING] Real Time Exceeded" + '\x1b[0m')
                self.restart_Sim()
                self.Done = True




        reward = self._CalcReward()

        return None,reward,self.Done,None
    
    def _CalcReward(self):

        

        self.reward_vals = [0,0,0,0,0]
        R_t = np.dot(self.reward_vals,list(self.reward_weights.values()))
        print(f"R_t_norm: {R_t/self.W_max:.3f} ")
        print(np.round(self.reward_vals,2))

        return R_t/self.W_max
    
    def Reward_Exp_Decay(self,x,threshold,k=5):
        if -0.1 < x < threshold:
            return 1
        elif threshold <= x:
            return np.exp(-k*(x-threshold))
        else:
            return 0
    
    def Reward_RotationDirection(self,x,rot_dir):

        if rot_dir == +1:
            return x if x < 0 else 1
        elif rot_dir == -1:
            return 1 if x < 0 else -x
        
    def Reward_ImpactAngle(self,Phi_deg,Phi_min,Impact_condition):

        if Impact_condition == -1:
            Phi_deg = -Phi_deg

        ## NOTE: THESE ARE ALL RELATIVE ANGLES
        Phi_TD = 180
        Phi_w = Phi_TD - Phi_min
        Phi_b = Phi_w/2

        if Phi_deg <= -2*Phi_min:
            return -1.0
        elif -2*Phi_min < Phi_deg <= Phi_min:
            return 0.5/(Phi_min - 0) * (Phi_deg - Phi_min) + 0.5
        elif Phi_min < Phi_deg <= Phi_min + Phi_b:
            return 0.5/((Phi_min + Phi_b) - Phi_min) * (Phi_deg - Phi_min) + 0.5
        elif Phi_min + Phi_b < Phi_deg <= Phi_TD:
            return -0.5/(Phi_TD - (Phi_min + Phi_b)) * (Phi_deg - Phi_TD) + 0.5
        elif Phi_TD < Phi_deg <= Phi_TD + Phi_b:
            return 0.5/((Phi_TD + Phi_b) - Phi_TD) * (Phi_deg - Phi_TD) + 0.5
        elif (Phi_TD + Phi_b) < Phi_deg <= (Phi_TD + Phi_w):
            return -0.5/((Phi_TD + Phi_w) - (Phi_TD + Phi_b)) * (Phi_deg - (Phi_TD + Phi_w)) + 0.5
        elif (Phi_TD + Phi_w) < Phi_deg <= (360 + 2*Phi_min):
            return -0.5/(360 - ((Phi_TD + Phi_w))) * (Phi_deg - ((Phi_TD + Phi_w))) + 0.5
        elif (360 + 2*Phi_min) <= Phi_deg:
            return -1.0

    def Reward_LT(self,CP_angle_deg,Leg_Num):

        if Leg_Num == 2:
            CP_angle_deg = -CP_angle_deg  # Reflect across the y-axis

        if -180 <= CP_angle_deg <= 0:
            return -np.sin(np.radians(CP_angle_deg))
        elif 0 < CP_angle_deg <= 180:
            return -1.0/180 * CP_angle_deg
        
    def Reward_GravityMoment(self,CP_angle_deg,Leg_Num):

        if Leg_Num == 2:
            CP_angle_deg = -CP_angle_deg  # Reflect across the y-axis

        return -np.sin(np.radians(CP_angle_deg))



if __name__ == "__main__":

    env = SAR_ParamOpt_Sim(GZ_Timeout=False)

    for ii in range(1000):
        Tau_trg = 0.30
        Rot_acc = -50
        Vel_d = 2.5
        Phi_d = 60
        env.ParamOptim_reset()
        obs,reward,done,info = env.ParamOptim_Flight(Tau_trg,Rot_acc,Vel_d,Phi_d)
        print(f"Ep: {ii} \t Reward: {reward:.02f} \t Reward_vec: ",end='')
        print(' '.join(f"{val:.2f}" for val in env.reward_vals))


