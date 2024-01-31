#!/usr/bin/env python3
import numpy as np
import time
import warnings
import math 
from sar_env import SAR_Sim_Interface

EPS = 1e-6 # Epsilon (Prevent division by zero)
COORD_FLIP = -1  # Swap sign to match proper coordinate notation


class SAR_ParamOpt_Sim(SAR_Sim_Interface):

    def __init__(self,GZ_Timeout=False,Ang_Acc_range=[-100,100],V_mag_range=[1.5,3.5],V_angle_range=[-175,-5],Plane_Angle_range=[0,180]):
        SAR_Sim_Interface.__init__(self,GZ_Timeout=GZ_Timeout)        

        ######################
        #    GENERAL CONFIGS
        ######################
        
        ## ENV CONFIG SETTINGS
        self.Env_Name = "SAR_ParamOptim_Env"
        self.GZ_Timeout = GZ_Timeout

        ## TESTING CONDITIONS     
        self.V_mag_range = V_mag_range  
        self.V_angle_range = V_angle_range
        self.Plane_Angle_range = Plane_Angle_range
        self.setAngAcc_range(Ang_Acc_range)

        ## TIME CONSTRAINTS
        self.t_rot_max = np.sqrt(np.radians(360)/np.abs(self.Ang_Acc_range[0])) # Allow enough time for a full rotation [s]
        self.t_trg_max = 1.0        # [s]
        self.t_impact_max = 1.0     # [s]
        self.t_run_max = 5.0        # [s]
        self.t_real_max = 15.0      # [s]

   
        ## INITIAL LEARNING/REWARD CONFIGS
        self.K_ep = 0
        self.Pol_Trg_Flag = False
        self.Done = False
        self.reward = 0
        self.reward_vals = np.array([0,0,0,0,0,0])
        self.reward_weights = {
            "W_Dist":0.5,
            "W_tau_cr":0.5,
            "W_LT":1.0,
            "W_GM":1.0,
            "W_Phi_rel":2.0,
            "W_Legs":2.0
        }
        self.W_max = sum(self.reward_weights.values())

        self.D_perp_min = np.inf
        self.Tau_trg = np.inf
        self.Tau_CR_trg = np.inf

        


    def ParamOptim_reset(self,V_mag=None,V_angle=None,Plane_Angle=None):

        ######################
        #    GENERAL CONFIGS
        ######################

        ## RESET LEARNING/REWARD CONDITIONS
        self.K_ep += 1
        self.Done = False
        self.Pol_Trg_Flag = False
        self.reward = 0

        self.D_perp_min = np.inf
        self.Tau_trg = np.inf
        self.Tau_CR_trg = np.inf

        self.Impact_Flag = False
        self.BodyContact_Flag = False
        self.Pad_Connections = 0
        self.MomentCutoff = False

        self.start_time_real = time.time()

        self.resetPose()


        ## SET PLANE POSE
        if Plane_Angle == None:

            Plane_Angle_Low = self.Plane_Angle_range[0]
            Plane_Angle_High = self.Plane_Angle_range[1]
            Plane_Angle = np.random.uniform(Plane_Angle_Low,Plane_Angle_High)
            self._setPlanePose(self.Plane_Pos,Plane_Angle)


        else:
            self._setPlanePose(self.Plane_Pos,Plane_Angle)


        ## SAMPLE VELOCITY AND FLIGHT ANGLE
        if V_mag == None or V_angle == None:
            V_mag,V_angle = self._sampleFlightConditions()

        else:
            V_mag = V_mag       # Flight velocity
            V_angle = V_angle   # Flight angle  

        self.V_mag = V_mag
        self.V_angle = V_angle

        ## CALC STARTING VELOCITY IN GLOBAL COORDS
        V_tx = V_mag*np.cos(np.deg2rad(V_angle))
        V_perp = V_mag*np.sin(np.deg2rad(V_angle))
        V_B_P = np.array([V_tx,0,V_perp]) # {t_x,n_p}
        V_B_O = self.R_PW(V_B_P,self.Plane_Angle_rad) # {X_W,Z_W}
        
        ## CALCULATE STARTING TAU VALUE
        self.Tau_CR_start = self.t_rot_max*2
        self.Tau_Body_start = (self.Tau_CR_start + self.Collision_Radius/V_perp) # Tau read by body

        ## CALC STARTING POSITION IN GLOBAL COORDS
        # (Derivation: Research_Notes_Book_3.pdf (9/17/23))

        r_P_O = np.array(self.Plane_Pos)                                        # Plane Position wrt to Origin - {X_W,Z_W}
        r_P_B = np.array([self.Tau_CR_start*V_tx,0,self.Tau_Body_start*V_perp])  # Body Position wrt to Plane - {t_x,n_p}
        r_B_O = r_P_O - self.R_PW(r_P_B,self.Plane_Angle_rad)                   # Body Position wrt to Origin - {X_W,Z_W}

        ## LAUNCH QUAD W/ DESIRED VELOCITY
        self.initial_state = (r_B_O,V_B_O)
        # self.resetPose(r_B_O[0],r_B_O[1],np.radians(-1),V_B_O[0],V_B_O[1],0)
        self.GZ_VelTraj(pos=r_B_O,vel=V_B_O)        


        # ## DOMAIN RANDOMIZATION (UPDATE INERTIA VALUES)
        # self.Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Iyy") + np.random.normal(0,1.5e-6)
        # self.Mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Mass") + np.random.normal(0,0.0005)
        # self.setModelInertia()


        ## RESET/UPDATE TIME CONDITIONS
        self.start_time_run = self._getTime()
        self.start_time_trg = np.nan
        self.start_time_impact = np.nan
        self.t_flight_max = self.Tau_Body_start*2   # [s]
        
        return self._get_obs(), {}

    def ParamOptim_Flight(self,Tau_trg,Rot_acc):

        ## RESET LOGGING CONDITIONS 
        OnceFlag_Trg = False    # Ensures Rot data recorded only once
        OnceFlag_Impact = False   # Ensures impact data recorded only once 
        self.sendCmd("Policy",cmd_vals=[Tau_trg,Rot_acc,0.0],cmd_flag=1)
        self.pausePhysics(pause_flag=False)


        while not self.Done: 

            t_now = self._getTime()

            ## RECORD LOWEST D_perp VALUE
            if self.D_perp < self.D_perp_min:
                self.D_perp_min = self.D_perp 

            ## START TRIGGER AND IMPACT TERMINATION TIMERS
            if (self.Trg_Flag == True and OnceFlag_Trg == False):
                self.start_time_trg = t_now     # Starts countdown for when to reset run
                OnceFlag_Trg = True             # Turns on to make sure this only runs once per rollout

            if (self.Impact_Flag_Ext and OnceFlag_Impact == False):
                self.start_time_impact = t_now
                OnceFlag_Impact = True

            # ============================
            ##    Termination Criteria 
            # ============================

            ## TRIGGER TIMEOUT  
            if (t_now-self.start_time_trg) > self.t_trg_max:
                self.error_str = "Run Completed: Pitch Timeout"
                self.Done = True
                # print(self.error_str)


            ## IMPACT TIMEOUT
            elif (t_now-self.start_time_impact) > self.t_impact_max:
                self.error_str = "Run Completed: Impact Timeout"
                self.Done = True
                # print(self.error_str)


            ## ROLLOUT TIMEOUT
            elif (t_now - self.start_time_run) > self.t_run_max:
                self.error_str = "Run Completed: Time Exceeded"
                self.Done = True
                # print(self.error_str)

            ## FREE FALL TERMINATION
            # elif self.vel[2] <= -0.5 and self.pos[2] <= 1.5: 
            #     self.error_str = "Run Completed: Falling Drone"
            #     self.Done = True
            #     # print(self.error_str)


            if (time.time() - self.start_time_real) > self.t_real_max and self.GZ_Timeout == True:
                print('\033[93m' + "[WARNING] Real Time Exceeded" + '\x1b[0m')
                self.Done = True
                self._restart_Sim()


        reward = self._CalcReward()

        return None,reward,self.Done,None
    
    def _CalcReward(self):

        if self.Impact_Flag_Ext:
            V_B_O_impact = self.R_PW(self.V_B_P_impact_Ext,self.Plane_Angle_rad)    # {X_W,Y_W,Z_W}
            V_hat_impact = V_B_O_impact/np.linalg.norm(V_B_O_impact)                # {X_W,Y_W,Z_W}

            temp_angle_rad = np.arctan2(V_hat_impact[0],V_hat_impact[2])
            temp_angle_deg = np.degrees(temp_angle_rad)

            if temp_angle_deg < 0:
                Phi_B_P_Impact_Condition = -1
            elif temp_angle_deg > 0:
                Phi_B_P_Impact_Condition = 1
            elif math.isnan(temp_angle_deg):
                Phi_B_P_Impact_Condition = 1

        else:
            ## CALC REWARD VALUES
            R_LT = 0
            R_GM = 0
            R_Phi = 0


        if self.ForelegContact_Flag:

            ## CALC IMPACT ANGLE CONDITIONS
            rot_dir = np.sign(self.Rot_Sum_impact_Ext)
            num_rev = self.Rot_Sum_impact_Ext // (rot_dir*360+1e-6)  # Integer division to get full revolutions
            Phi_B_O_impact_deg = self.Eul_B_O_impact_Ext[1] + rot_dir*(num_rev*360)

            Phi_B_P_impact_deg = Phi_B_O_impact_deg - self.Plane_Angle_deg
            Phi_P_B_impact_deg = -Phi_B_P_impact_deg

            Beta1_deg = -Phi_P_B_impact_deg - self.Gamma_eff + 90
            Beta1_rad = np.radians(Beta1_deg)

            ## CALC LEG DIRECTION VECTOR
            r_B_C1 = np.array([-self.L_eff,0,0]) # {e_r1,0,e_beta1}
            r_B_C1 = self.R_PW(self.R_C1P(r_B_C1,Beta1_rad),self.Plane_Angle_rad)   # {X_W,Y_W,Z_W}

            r_C1_B = -r_B_C1                        # {X_W,Y_W,Z_W}
            e_r_hat = r_C1_B/np.linalg.norm(r_C1_B) # {X_W,Y_W,Z_W}

            ## MOMENTUM TRANSFER REWARD
            CP_LT = np.cross(V_hat_impact,e_r_hat) # {X_W,Y_W,Z_W}
            DP_LT = np.dot(V_hat_impact,e_r_hat)
            CP_LT_angle_deg = np.degrees(np.arctan2(CP_LT,DP_LT))[1]
            R_LT = self.Reward_LT(CP_LT_angle_deg,self.ForelegContact_Flag,self.HindlegContact_Flag)

            
            ## GRAVITY MOMENT REWARD
            g_hat = np.array([0,0,-1])              # {X_W,Y_W,Z_W}
            CP_GM = np.cross(g_hat,e_r_hat)         # {X_W,Y_W,Z_W}
            DP_GM = np.dot(g_hat,e_r_hat)
            CP_GM_angle_deg = np.degrees(np.arctan2(CP_GM,DP_GM))[1]
            R_GM = self.Reward_GravityMoment(CP_GM_angle_deg,self.ForelegContact_Flag,self.HindlegContact_Flag)

            ## PHI IMPACT REWARD
            R_Phi = self.Reward_ImpactAngle(Phi_P_B_impact_deg,self.Phi_P_B_impact_Min_deg,Phi_B_P_Impact_Condition)

        elif self.HindlegContact_Flag:

            ## CALC IMPACT ANGLE CONDITIONS
            rot_dir = np.sign(self.Rot_Sum_impact_Ext)
            num_rev = self.Rot_Sum_impact_Ext // (rot_dir*360+1e-6)  # Integer division to get full revolutions
            Phi_B_O_impact_deg = self.Eul_B_O_impact_Ext[1] + rot_dir*(num_rev*360)

            Phi_B_P_impact_deg = Phi_B_O_impact_deg - self.Plane_Angle_deg
            Phi_P_B_impact_deg = -Phi_B_P_impact_deg

            Beta2_deg = self.Gamma_eff + -Phi_P_B_impact_deg  + 90
            Beta2_rad = np.radians(Beta2_deg)

            ## CALC LEG DIRECTION VECTOR
            r_B_C2 = np.array([-self.L_eff,0,0]) # {e_r2,0,e_beta2}
            r_B_C2 = self.R_PW(self.R_C2P(r_B_C2,Beta2_rad),self.Plane_Angle_rad)   # {X_W,Y_W,Z_W}

            r_C2_B = -r_B_C2                        # {X_W,Y_W,Z_W}
            e_r_hat = r_C2_B/np.linalg.norm(r_C2_B) # {X_W,Y_W,Z_W}

            ## MOMENTUM TRANSFER REWARD
            CP_LT = np.cross(V_hat_impact,e_r_hat) # {X_W,Y_W,Z_W}
            DP_LT = np.dot(V_hat_impact,e_r_hat)
            CP_LT_angle_deg = np.degrees(np.arctan2(CP_LT,DP_LT))[1]
            R_LT = self.Reward_LT(CP_LT_angle_deg,self.ForelegContact_Flag,self.HindlegContact_Flag)

            
            ## GRAVITY MOMENT REWARD
            g_hat = np.array([0,0,-1])              # {X_W,Y_W,Z_W}
            CP_GM = np.cross(g_hat,e_r_hat)         # {X_W,Y_W,Z_W}
            DP_GM = np.dot(g_hat,e_r_hat)
            CP_GM_angle_deg = np.degrees(np.arctan2(CP_GM,DP_GM))[1]
            R_GM = self.Reward_GravityMoment(CP_GM_angle_deg,self.ForelegContact_Flag,self.HindlegContact_Flag)

            ## PHI IMPACT REWARD
            R_Phi = self.Reward_ImpactAngle(Phi_P_B_impact_deg,self.Phi_P_B_impact_Min_deg,Phi_B_P_Impact_Condition)

        elif self.BodyContact_Flag:

            ## CALC IMPACT ANGLE CONDITIONS
            rot_dir = np.sign(self.Rot_Sum_impact_Ext)
            num_rev = self.Rot_Sum_impact_Ext // (rot_dir*360+1e-6)  # Integer division to get full revolutions
            Phi_B_O_impact_deg = self.Eul_B_O_impact_Ext[1] + rot_dir*(num_rev*360)

            Phi_B_P_impact_deg = Phi_B_O_impact_deg - self.Plane_Angle_deg
            Phi_P_B_impact_deg = -Phi_B_P_impact_deg

            ## CALC REWARD VALUES
            R_LT = 0
            R_GM = 0
            R_Phi = self.Reward_ImpactAngle(Phi_P_B_impact_deg,self.Phi_P_B_impact_Min_deg,Phi_B_P_Impact_Condition)


        ## REWARD: MINIMUM DISTANCE 
        if self.Tau_CR_trg < np.inf:
            R_dist = self.Reward_Exp_Decay(self.D_perp_min,0)
        else:
            R_dist = 0

        ## REWARD: TAU_CR TRIGGER
        R_tau_cr = self.Reward_Exp_Decay(self.Tau_CR_trg,0.2)
        # R_tau_cr = 0

        ## REWARD: PAD CONNECTIONS
        if self.Pad_Connections >= 3: 
            R_Legs = 1.0
        elif self.Pad_Connections == 2:
            R_Legs = 0.2
        else:
            R_Legs = 0.0

        if self.BodyContact_Flag:
            R_Legs = R_Legs*0.5

        self.reward_vals = [R_dist,R_tau_cr,R_LT,R_GM,R_Phi,R_Legs]
        R_t = np.dot(self.reward_vals,list(self.reward_weights.values()))
        print(f"R_t_norm: {R_t/self.W_max:.3f}")
        print(np.round(self.reward_vals,2))

        return R_t/self.W_max
    
    def Reward_Exp_Decay(self,x,threshold,k=5):
        if -0.1 < x < threshold:
            return 1
        elif threshold <= x:
            return np.exp(-k*(x-threshold))
        else:
            return 0
    
        
    def Reward_ImpactAngle(self,Phi_deg,Phi_min,Impact_condition):

        if Impact_condition == -1:
            Phi_deg = -Phi_deg

        ## NOTE: THESE ARE ALL RELATIVE ANGLES
        Phi_TD = 180
        Phi_w = Phi_TD - Phi_min
        Phi_b = Phi_w/2

        if Phi_deg <= -2*Phi_min:
            return 0.0
        elif -2*Phi_min < Phi_deg <= Phi_min:
            return 0.5/(3*Phi_min - 0) * (Phi_deg - Phi_min) + 0.50
        elif Phi_min < Phi_deg <= Phi_min + Phi_b:
            return 0.5/((Phi_min + Phi_b) - Phi_min) * (Phi_deg - Phi_min) + 0.5
        elif Phi_min + Phi_b < Phi_deg <= Phi_TD:
            return -0.25/(Phi_TD - (Phi_min + Phi_b)) * (Phi_deg - Phi_TD) + 0.75
        elif Phi_TD < Phi_deg <= Phi_TD + Phi_b:
            return 0.25/((Phi_TD + Phi_b) - Phi_TD) * (Phi_deg - Phi_TD) + 0.75
        elif (Phi_TD + Phi_b) < Phi_deg <= (Phi_TD + Phi_w):
            return -0.5/((Phi_TD + Phi_w) - (Phi_TD + Phi_b)) * (Phi_deg - (Phi_TD + Phi_w)) + 0.5
        elif (Phi_TD + Phi_w) < Phi_deg <= (360 + 2*Phi_min):
            return -0.5/(3*Phi_min) * (Phi_deg - ((Phi_TD + Phi_w))) + 0.5
        elif (360 + 2*Phi_min) <= Phi_deg:
            return 0.0

    def Reward_LT(self,CP_angle_deg,ForelegContact_Flag,HindlegContact_Flag):

        if HindlegContact_Flag == True:
            CP_angle_deg = -CP_angle_deg  # Reflect function across the y-axis
        elif ForelegContact_Flag == True:
            CP_angle_deg = CP_angle_deg
        else:
            return 0

        ## CALCULATE REWARD
        if -180 <= CP_angle_deg <= 0:
            return -np.sin(np.radians(CP_angle_deg)) * 1/2 + 0.5
        elif 0 < CP_angle_deg <= 180:
            return -1.0/180 * CP_angle_deg * 1/2 + 0.5
        
    def Reward_GravityMoment(self,CP_angle_deg,ForelegContact_Flag,HindlegContact_Flag):

        if HindlegContact_Flag == True:
            CP_angle_deg = -CP_angle_deg  # Reflect function across the y-axis
        elif ForelegContact_Flag == True:
            CP_angle_deg = CP_angle_deg
        else:
            return 0

        ## CALCULATE REWARD
        return -np.sin(np.radians(CP_angle_deg)) * 1/2 + 0.5



if __name__ == "__main__":

    env = SAR_ParamOpt_Sim(GZ_Timeout=False)

    for ii in range(1000):
        Tau_trg = 0.39
        Rot_acc = 200
        V_mag = 2.5
        V_angle = 120
        Plane_Angle = 0
        env.ParamOptim_reset(V_mag=V_mag,V_angle=V_angle,Plane_Angle=Plane_Angle)
        obs,reward,done,info = env.ParamOptim_Flight(Tau_trg,Rot_acc)
        # print(f"Ep: {ii} \t Reward: {reward:.02f} \t Reward_vec: ",end='')
        # print(' '.join(f"{val:.2f}" for val in env.reward_vals))


