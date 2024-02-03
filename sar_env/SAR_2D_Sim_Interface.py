#!/usr/bin/env python3
import numpy as np
import os
import rospy

import numpy as np
from sar_env import SAR_Base_Interface

import time
import pygame as pg
import os
import rospy
## ROS MESSAGES AND SERVICES

## DEFINE COLORS
WHITE_PG = (255,255,255)
GREY_PG = (200,200,200)
BLACK_PG = (0,0,0)
RED_PG = (204,0,0)
BLUE_PG = (29,123,243)
SKY_BLUE_PG = (0,153,153)
GREEN_PG = (0,153,0)
PURPLE_PG = (76,0,153)
ORANGE_PG = (255,128,0)

EPS = 1e-6 # Epsilon (Prevent division by zero)
COORD_FLIP = -1  # Swap sign to match proper coordinate notation

EPS = 1e-6 # Epsilon (Prevent division by zero)
YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = '\033[34m'  
RESET = '\033[0m'  # Reset to default color

RAD2DEG = 180.0/np.pi
DEG2RAD = np.pi/180.0

class SAR_2D_Sim_Interface(SAR_Base_Interface):

    def __init__(self,Render=True):
        SAR_Base_Interface.__init__(self)

        self.I_c = self.Ref_Iyy + self.Ref_Mass*self.L_eff**2
        self.initial_state = (np.array([0,0,0]),np.array([0,0,0]))

        ## PLANE PARAMETERS
        self.r_P_O = [1,0,0]
        self.Plane_Angle_deg = 0
        self.Plane_Angle_rad = np.radians(self.Plane_Angle_deg)

        ## INITIAL LEARNING/REWARD CONFIGS
        self.Trg_Flag = False
        self.Impact_Flag_Ext = False

        self.BodyContact_Flag = False
        self.ForelegContact_Flag = False
        self.HindlegContact_Flag = False
        self.Pad_Connections = 0


        self.MomentCutoff = False
        

        ## SPECIAL CONFIGS
        self.State = [np.zeros(3),0,np.zeros(3),0]
        self.State_Trg = [np.full(3,np.nan),np.nan,np.full(3,np.nan),np.nan]
        self.State_Impact = [np.full(3,np.nan),np.nan,np.full(3,np.nan),np.nan]




        ## PHYSICS PARAMETERS
        self.g = 9.81       # Gravity [m/s^2]
        self.dt = 1e-3      # [s]
        self.t = 0          # [s]

        ## RENDERING PARAMETERS
        self.world_width = 3.0      # [m]
        self.world_height = 2.0     # [m]
        self.x_offset = 1           # [m]
        self.y_offset = 1           # [m]
        self.screen_width = 1000    # [pixels]
        self.screen_height = self.screen_width*self.world_height/self.world_width # [pixels]
        
        self.RENDER = Render
        self.screen = None
        self.clock = None
        self.isopen = True

    def _getTime(self):

        return self.t
    
    def _get_pose(self):

        r_B_O,Phi_B_O,_,_ = self._getState()

        ## LEG COORDS
        L1 = np.array([ self.L_eff*np.sin(self.Gamma_eff*DEG2RAD),0,-self.L_eff*np.cos(self.Gamma_eff*DEG2RAD)])
        L2 = np.array([-self.L_eff*np.sin(self.Gamma_eff*DEG2RAD),0,-self.L_eff*np.cos(self.Gamma_eff*DEG2RAD)])

        ## PROP COORDS
        Prop1 = np.array([ self.Forward_Reach,0,0])
        Prop2 = np.array([-self.Forward_Reach,0,0])

        ## CONVERT BODY COORDS TO WORLD COORDS
        L1 = r_B_O + self.R_BW(L1,Phi_B_O)
        L2 = r_B_O + self.R_BW(L2,Phi_B_O)
        Prop1 = r_B_O + self.R_BW(Prop1,Phi_B_O)
        Prop2 = r_B_O + self.R_BW(Prop2,Phi_B_O)

        return np.array([r_B_O,L1,L2,Prop1,Prop2])

    def _checkImpactConditions(self,x,z,Phi_B_O):

        r_B_O,Leg1_Pos,Leg2_Pos,Prop1_Pos,Prop2_Pos = self._get_pose()
        r_P_O = self.r_P_O

        ## CHECK FOR CG CONTACT 
        r_B_P = self.R_WP((r_B_O - r_P_O),self.Plane_Angle_rad)
        if r_B_P[1] >= 0:
            impact_flag = True
            Body_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR PROP CONTACT
        for Prop_Pos in [Prop1_Pos,Prop2_Pos]:

            Prop_wrt_Plane = self.R_WP((Prop_Pos - r_P_O),self.Plane_Angle_rad)
            if Prop_wrt_Plane[1] >= 0:
                impact_flag = True
                Body_contact = True

                return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR LEG1 CONTACT
        Leg1_wrt_Plane = self.R_WP((Leg1_Pos - self.r_P_O),self.Plane_Angle_rad)
        if Leg1_wrt_Plane[1] >= 0:
            impact_flag = True
            Leg1_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR LEG2 CONTACT
        Leg2_wrt_Plane = self.R_WP((Leg2_Pos - self.r_P_O),self.Plane_Angle_rad)
        if Leg2_wrt_Plane[1] >= 0:
            impact_flag = True
            Leg2_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]


        return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

    def _setState(self,r_B_O,Phi_B_O,V_B_O,dPhi):

        self.State = (r_B_O,Phi_B_O,V_B_O,dPhi)

    def _getState(self):

        return self.State
    
    def _getObs(self):
        
        ## PLANE POSITION AND UNIT VECTORS
        r_B_O,Phi_B_O,V_B_O,dPhi = self._getState()
        r_P_O = self.r_P_O


        ## CALC DISPLACEMENT FROM PLANE CENTER
        r_P_B = r_P_O - r_B_O # {X_W,Z_W}

        ## CALC RELATIVE DISTANCE AND VEL
        D_perp = self.R_WP(r_P_B,self.Plane_Angle_rad)[2]
        V_tx,_,V_perp = self.R_WP(V_B_O,self.Plane_Angle_rad)

        ## CALC OPTICAL FLOW VALUES
        Tau = np.clip(D_perp/(V_perp + EPS),0,5)
        Theta_x = np.clip(V_tx/(D_perp + EPS),-20,20)


        r_CR_B = np.array([0,0,self.Collision_Radius])                        # {t_x,n_p}
        r_P_CR = r_P_O - (r_B_O + self.R_PW((r_CR_B),self.Plane_Angle_rad)) # {X_W,Z_W}
        D_perp_CR = self.R_WP(r_P_CR,self.Plane_Angle_rad)[2]               # Convert to plane coords - {t_x,n_p}
        Tau_CR = np.clip(D_perp_CR/(V_perp + EPS),-5,5)


        ## OBSERVATION VECTOR
        obs = np.array([Tau_CR,Theta_x,D_perp,self.Plane_Angle_rad],dtype=np.float32)

        return obs
    
    def resetPose(self,z_0=0.4):
        self._setModelState()

    def Sim_VelTraj(self,pos,vel): 
        self._setState(pos,0,vel,0)
        

    def _setModelState(self,pos=[0,0,0.4],quat=[0,0,0,1],vel=[0,0,0],ang_vel=[0,0,0]):

        pass

    def _setPlanePose(self,Pos,Plane_Angle):
        self.Plane_Pos = Pos
        self.Plane_Angle_deg = Plane_Angle

    
    
    def Step(self,a_Rot):


        ## CHECK FOR IMPACT
        x,z,Phi_B_O,vx,vz,dPhi_B_O = self._getState()
        self.Impact_Flag_Ext,self.Impact_Conditions = self._checkImpactConditions(x,z,Phi_B_O)

        ## NO IMPACT
        if self.Impact_Flag_Ext == False:

            ## UPDATE ROTATION FLIGHT STEP
            self._iter_step_Rot(a_Rot)

            # UPDATE MINIMUM DISTANCE
            D_perp = self._getObs()[2]
            if D_perp <= self.D_perp_min:
                self.D_perp_min = D_perp 

            # UPDATE RENDER
            if self.RENDER:
                self.render()

        if self.Impact_Flag_Ext == True:


            ## GRAB IMPACT STATE
            (BodyContact_Flag,ForelegContact_Flag,HindlegContact_Flag) = self.Impact_Conditions
            self.State_Impact = self._getState()

            ## BODY CONTACT
            if BodyContact_Flag == True:
                self.BodyContact_Flag = True
                self.Pad_Connections = 0
                self.Done = True

            ## LEG 1 CONTACT
            elif ForelegContact_Flag == True:

                Beta_1,dBeta_1 = self._impact_conversion(self.State,self.params,self.Impact_Conditions)

                ## FIND IMPACT COORDS (wrt World-Coords)
                r_C1_O = self._get_pose()[1]

                while not self.Done:

                    ## ITERATE THROUGH SWING
                    Beta_1,dBeta_1 = self._iter_step_Swing(Beta_1,dBeta_1,impact_leg=1)

                    ## CHECK FOR END CONDITIONS
                    if Beta_1 <= -self._beta_landing(impact_leg=1):
                        self.BodyContact_Flag = False
                        self.Pad_Connections = 4
                        self.Done = True

                    elif Beta_1 >= -self._beta_prop(impact_leg=1):
                        self.BodyContact_Flag = True
                        self.Pad_Connections = 2
                        self.Done = True

                    elif self.t - start_time_impact >= self.t_impact_max:
                        self.BodyContact_Flag = False
                        self.Pad_Connections = 2
                        self.Done = True

                    ## CONVERT BODY BACK TO WORLD COORDS
                    r_B_C1 = np.array([-L_eff,0])                                           # {e_r1,e_beta1}
                    r_B_C1 = self.R_PW(self.R_C1P(r_B_C1,Beta_1),self.Plane_Angle_rad)  # {X_W,Z_W}
                    r_B_O = r_C1_O + r_B_C1                                             # {X_W,Z_W}

                    v_B_C1 = np.array([0,L_eff*dBeta_1])                                    # {e_r1,e_beta1}
                    v_B_C1 = self.R_PW(self.R_C1P(v_B_C1,Beta_1),self.Plane_Angle_rad)  # {X_W,Z_W}

                    Phi_B_O = np.arctan2(-np.cos(Beta_1 + Gamma_eff_rad + self.Plane_Angle_rad), \
                                        np.sin(Beta_1 + Gamma_eff_rad + self.Plane_Angle_rad))
                    self.State = (r_B_O[0],r_B_O[1],Phi_B_O,v_B_C1[0],v_B_C1[1],0)
                    

                    if self.RENDER:
                        self.render()


            ## LEG 2 CONTACT
            elif HindlegContact_Flag == True:

                Beta_2,dBeta_2 = self._impact_conversion(self.State,self.params,self.Impact_Conditions)

                ## FIND IMPACT COORDS (wrt World-Coords)
                r_C2_O = self._get_pose()[2]

                while not self.Done:

                    ## ITERATE THROUGH SWING
                    Beta_2,dBeta_2 = self._iter_step_Swing(Beta_2,dBeta_2,impact_leg=2)

                    ## CHECK FOR END CONDITIONS
                    if Beta_2 >= -self._beta_landing(impact_leg=2):
                        self.BodyContact_Flag = False
                        self.Pad_Connections = 4
                        self.Done = True

                    elif Beta_2 <= -self._beta_prop(impact_leg=2):
                        self.BodyContact_Flag = True
                        self.Pad_Connections = 2
                        self.Done = True

                    elif self.t - start_time_impact >= self.t_impact_max:
                        self.BodyContact_Flag = False
                        self.Pad_Connections = 2
                        self.Done = True

                    ## CONVERT BODY BACK TO WORLD COORDS
                    r_B_C2 = np.array([-L_eff,0])                                           # {e_r1,e_beta1}
                    r_B_C2 = self.R_PW(self.R_C2P(r_B_C2,Beta_2),self.Plane_Angle_rad)  # {X_W,Z_W}
                    r_B_O = r_C2_O + r_B_C2                                             # {X_W,Z_W}

                    v_B_C2 = np.array([0,L_eff*dBeta_2])                                    # {e_r1,e_beta1}
                    v_B_C2 = self.R_PW(self.R_C2P(v_B_C2,Beta_2),self.Plane_Angle_rad)  # {X_W,Z_W}

                    Phi_B_O = np.arctan2(-np.cos(Beta_2 - Gamma_eff_rad + self.Plane_Angle_rad), \
                                        np.sin(Beta_2 - Gamma_eff_rad + self.Plane_Angle_rad))
                    self.State = (r_B_O[0],r_B_O[1],Phi_B_O,v_B_C2[0],v_B_C2[1],0)

                    ## UPDATE MINIMUM DISTANCE
                    D_perp = self._getObs()[2]
                    if not self.Done:
                        if D_perp <= self.D_perp_min:
                            self.D_perp_min = D_perp 

                    if self.RENDER:
                        self.render()

    def render(self):

        ## SET DEFAULT WINDOW POSITION
        Win_Loc_z = 500
        Win_Loc_y = 700
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (Win_Loc_z,Win_Loc_y)

        ## CONVERT COORDINATES TO PIXEL LOCATION
        def c2p(Pos):

            if len(Pos) == 2:
                x = Pos[0]
                y = Pos[1]

            elif len(Pos) == 3:
                x = Pos[0]
                y = Pos[2]

            
            scale_x = self.screen_width/self.world_width
            scale_y = self.screen_height/self.world_height
            
            x_p = (self.x_offset+x)*scale_x # [pixels]
            y_p = (self.y_offset+y)*scale_y # [pixels]
            
            return (x_p,y_p)
        
        ## INITIATE SCREEN AND CLOCK ON FIRST LOADING
        if self.screen is None:
            pg.init()
            self.screen = pg.display.set_mode((self.screen_width,self.screen_height))
        
        if self.clock is None:
            self.clock = pg.time.Clock()


        ## GET CURRENT STATE
        r_B_O,Phi_B_O,V_B_O,dPhi_B_O = self._getState()

        
        ## CREATE BACKGROUND SURFACE
        self.surf = pg.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE_PG)

        ## TRAJECTORY LINE
        r_B_O,V_B_O = self.initial_state
        self.draw_line_dashed(self.surf,GREY_PG,c2p(r_B_O - 5*V_B_O),c2p(r_B_O + 5*V_B_O),width=3)

        ## ORIGIN AXES
        pg.draw.line(self.surf,GREEN_PG,c2p((0,0,0)),c2p((0.1,0,0)),width=5) # X_w   
        pg.draw.line(self.surf,BLUE_PG, c2p((0,0,0)),c2p((0,0,0.1)),width=5) # Z_w   
        pg.draw.circle(self.surf,RED_PG,c2p((0,0,0)),radius=4,width=0)


        ## LANDING SURFACE
        pg.draw.line(self.surf,GREY_PG,
                         c2p(self.r_P_O + self.R_PW(np.array([-2,0,0]),self.Plane_Angle_rad)),
                         c2p(self.r_P_O + self.R_PW(np.array([+2,0,0]),self.Plane_Angle_rad)),width=2)
        
        pg.draw.line(self.surf,BLACK_PG,
                         c2p(self.r_P_O + self.R_PW(np.array([-0.5,0,0]),self.Plane_Angle_rad)),
                         c2p(self.r_P_O + self.R_PW(np.array([+0.5,0,0]),self.Plane_Angle_rad)),width=5)
    
        ## LANDING SURFACE AXES
        pg.draw.line(self.surf,GREEN_PG,c2p(self.r_P_O),c2p(self.r_P_O + self.R_PW(np.array([0.1,0,0]),self.Plane_Angle_rad)),width=7)  # t_x   
        pg.draw.line(self.surf,BLUE_PG, c2p(self.r_P_O),c2p(self.r_P_O + self.R_PW(np.array([0,0,0.1]),self.Plane_Angle_rad)),width=7)  # n_p 
        pg.draw.circle(self.surf,RED_PG,c2p(self.r_P_O),radius=4,width=0)


        ## DRAW QUADROTOR
        Pose = self._get_pose()
        pg.draw.line(self.surf,RED_PG,c2p(Pose[0]),c2p(Pose[1]),width=3) # Leg 1
        pg.draw.line(self.surf,BLACK_PG,c2p(Pose[0]),c2p(Pose[2]),width=3) # Leg 2
        pg.draw.line(self.surf,BLACK_PG,c2p(Pose[0]),c2p(Pose[3]),width=3) # Prop 1
        pg.draw.line(self.surf,BLACK_PG,c2p(Pose[0]),c2p(Pose[4]),width=3) # Prop 2
        pg.draw.circle(self.surf,GREY_PG,c2p(Pose[0]),radius=self.Collision_Radius*self.screen_width/self.world_width,width=2)

        ## BODY AXES
        pg.draw.line(self.surf,GREEN_PG,c2p(Pose[0]),c2p(Pose[0] + self.R_BW(np.array([0.05,0,0]),Phi_B_O)),width=5)  # B_x   
        pg.draw.line(self.surf,BLUE_PG,c2p(Pose[0]),c2p(Pose[0] + self.R_BW(np.array([0,0,0.05]),Phi_B_O)),width=5)  # B_z  

        ## GRAVITY UNIT VECTOR
        g_hat = np.array([0,0,-1])
        pg.draw.line(self.surf,PURPLE_PG,c2p(Pose[0]),c2p(Pose[0] + g_hat*0.1),width=3)

        ## VELOCITY UNIT VECTOR
        v_hat = V_B_O/np.linalg.norm(V_B_O)
        pg.draw.line(self.surf,ORANGE_PG,c2p(Pose[0]),c2p(Pose[0] + v_hat*0.1),width=3)




        ## TRIGGER INDICATOR
        if self.Trg_Flag == True:
            pg.draw.circle(self.surf,RED_PG,  c2p(Pose[0]),radius=4,width=0)
            pg.draw.circle(self.surf,BLACK_PG,c2p(Pose[0]),radius=5,width=3)
        else:
            pg.draw.circle(self.surf,BLUE_PG, c2p(Pose[0]),radius=4,width=0)
            pg.draw.circle(self.surf,BLACK_PG,c2p(Pose[0]),radius=5,width=3)



        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pg.transform.flip(self.surf, False, True)

        ## WINDOW TEXT
        my_font = pg.font.SysFont(None, 30)

        # ## STATES TEXT
        # text_States = my_font.render(f'States:', True, GREY_PG)
        # text_t_step = my_font.render(f'Time Step: {self.t:7.03f} [s]', True, BLACK_PG)
        # text_V_mag = my_font.render(f'V_mag: {self.V_mag:.2f} [m/s]', True, BLACK_PG)
        # text_Rel_Angle = my_font.render(f'Flight_Angle: {self.V_angle:.2f} [deg]', True, BLACK_PG)

        # ## OBSERVATIONS TEXT
        # text_Obs = my_font.render(f'Observations:', True, GREY_PG)
        # text_Tau = my_font.render(f'Tau: {self._getObs()[0]:2.2f} [s]', True, BLACK_PG)
        # text_theta_x = my_font.render(f'Theta_x: {self._getObs()[1]:2.2f} [rad/s]', True, BLACK_PG)
        # text_D_perp = my_font.render(f'D_perp: {self._getObs()[2]:2.2f} [m]', True, BLACK_PG)
        # text_Plane_Angle = my_font.render(f'Plane Angle: {self.Plane_Angle_deg:3.1f} [deg]', True, BLACK_PG)

        # ## ACTIONS TEXT
        # text_Actions = my_font.render(f'Actions:', True, GREY_PG)
        # # text_Trg_Action = my_font.render(f'Trg_Action: {self.action_trg[0]:3.1f}', True, BLACK_PG)
        # # text_Rot_Action = my_font.render(f'Rot_Action: {self.action_trg[1]:3.1f}', True, BLACK_PG)

        # ## OTHER TEXT
        # text_Other = my_font.render(f'Other:', True, GREY_PG)
        # # text_reward = my_font.render(f'Prev Reward: {self.reward:.3f}',True, BLACK_PG)
        # text_Tau_trg = my_font.render(f'Tau_trg: {self.Tau_trg:.3f} [s]',True, BLACK_PG)
        # text_Tau_CR_trg = my_font.render(f'Tau_CR_trg: {self.Tau_CR_trg:.3f} [s]',True, BLACK_PG)
        # text_Tau_CR = my_font.render(f'Tau CR: {self.Tau_CR:.3f} [s]',True, BLACK_PG)
        # text_Phi = my_font.render(f'Phi: {np.degrees(Phi_B_O):.0f} deg',True, BLACK_PG)





        ## DRAW OBJECTS TO SCREEN
        self.screen.blit(self.surf,         (0,0))
        # self.screen.blit(text_States,       (5,5))
        # self.screen.blit(text_t_step,       (5,5 + 25*1))
        # self.screen.blit(text_Rel_Angle,    (5,5 + 25*2))
        # self.screen.blit(text_V_mag,        (5,5 + 25*3))

        # self.screen.blit(text_Obs,          (5,5 + 25*5))
        # self.screen.blit(text_Tau,          (5,5 + 25*6))
        # self.screen.blit(text_theta_x,      (5,5 + 25*7))
        # self.screen.blit(text_D_perp,       (5,5 + 25*8))
        # self.screen.blit(text_Plane_Angle,  (5,5 + 25*9))

        # self.screen.blit(text_Actions,      (5,5 + 25*11))
        # # self.screen.blit(text_Trg_Action,   (5,5 + 25*12))
        # # self.screen.blit(text_Rot_Action,   (5,5 + 25*13))

        # self.screen.blit(text_Other,        (5,5 + 25*15))
        # # self.screen.blit(text_reward,       (5,5 + 25*16))
        # self.screen.blit(text_Tau_trg,      (5,5 + 25*17))
        # self.screen.blit(text_Tau_CR_trg,   (5,5 + 25*18))
        # self.screen.blit(text_Tau_CR,       (5,5 + 25*19))
        # self.screen.blit(text_Phi,          (5,5 + 25*20))






        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(30) # [Hz]
        pg.display.flip()

    def close(self):

        return
    


    def _iterStep(self,n_steps=10):

        for _ in range(n_steps):

            if self.Trg_Flag == False:
                self._iterStep_Flight()

                # # CHECK FOR IMPACT
            # x,z,phi_B_O,Vx,Vz,dphi_B_O = self._getState()
            # self.Impact_Flag_Ext,Impact_Conditions = self._checkImpactConditions(x,z,phi_B_O)

            # # UPDATE MINIMUM DISTANCE
            # D_perp = next_obs[2]
            # if D_perp <= self.D_perp_min:
            #     self.D_perp_min = D_perp 

                

    def _iterStep_Flight(self):

        ## UPDATE STATE
        r_B_O,Phi_B_O,V_B_O,dPhi_B_O = self._getState()

        self.t += self.dt

        x_acc = 0.0
        r_B_O[0] = r_B_O[0] + self.dt*V_B_O[0]
        V_B_O[0] = V_B_O[0] + self.dt*x_acc

        z_acc = 0.0
        r_B_O[2] = r_B_O[2] + self.dt*V_B_O[2]
        V_B_O[2] = V_B_O[2] + self.dt*z_acc

        phi_acc = 0.0
        Phi_B_O = Phi_B_O + self.dt*dPhi_B_O
        dPhi_B_O = dPhi_B_O + self.dt*phi_acc

        self._setState(r_B_O,Phi_B_O,V_B_O,dPhi_B_O)


    def _iter_step_Rot(self,Rot_action,n_steps=2):

        ## PARAMS
        self.Gamma_eff,self.L_eff,self.Forward_Reach,M,Iyy,I_c = self.params

        ## CURRENT STATE
        x,z,phi,vx,vz,dphi = self._getState()

        ## TURN OFF BODY MOMENT IF ROTATED PAST 90 DEG
        if np.abs(phi) < np.deg2rad(90) and self.MomentCutoff == False:
            My = Rot_action

        else: 
            self.MomentCutoff= True
            My = 0

        for _ in range(n_steps):

            ## STEP UPDATE
            self.t += self.dt

            z_acc = -self.g
            z = z + self.dt*vz
            vz = vz + self.dt*z_acc

            x_acc = 0
            x = x + self.dt*vx
            vx = vx + self.dt*x_acc

            phi_acc = My/Iyy
            phi = phi + self.dt*dphi
            dphi = dphi + self.dt*phi_acc

            self.State = np.array([x,z,phi,vx,vz,dphi])

    def _iter_step_Swing(self, Beta, dBeta, impact_leg, n_steps=2):


        self.Gamma_eff, self.L_eff, self.Forward_Reach, M, Iyy, I_c = self.params

        for _ in range(n_steps):
            if impact_leg == 1:
                ## GRAVITY MOMENT
                M_g = -M * self.g * self.L_eff * np.cos(Beta) * np.cos(self.Plane_Angle_rad) \
                    + M * self.g * self.L_eff * np.sin(Beta) * np.sin(self.Plane_Angle_rad)
            if impact_leg == 2:
                ## GRAVITY MOMENT
                M_g = -M * self.g * self.L_eff * np.cos(Beta) * np.cos(self.Plane_Angle_rad) \
                    + M * self.g * self.L_eff * np.sin(Beta) * np.sin(self.Plane_Angle_rad)

            ## ITER STEP BETA
            self.t += self.dt
            Beta_acc = M_g / I_c
            Beta = Beta + self.dt * dBeta
            dBeta = dBeta + self.dt * Beta_acc

        return Beta, dBeta
    def _iter_step_Swing(self,Beta,dBeta,impact_leg,n_steps=2):

        self.Gamma_eff,self.L_eff,self.Forward_Reach,M,Iyy,I_c = self.params

        for _ in range(n_steps):

            if impact_leg==1:

                ## GRAVITY MOMENT
                M_g = -M*self.g*self.L_eff*np.cos(Beta)*np.cos(self.Plane_Angle_rad) \
                        + M*self.g*self.L_eff*np.sin(Beta)*np.sin(self.Plane_Angle_rad)
                
            if impact_leg==2:

                ## GRAVITY MOMENT
                M_g = -M*self.g*self.L_eff*np.cos(Beta)*np.cos(self.Plane_Angle_rad) \
                        + M*self.g*self.L_eff*np.sin(Beta)*np.sin(self.Plane_Angle_rad)
                
                
            ## ITER STEP BETA
            self.t += self.dt
            Beta_acc = M_g/I_c
            Beta = Beta + self.dt*dBeta
            dBeta = dBeta + self.dt*Beta_acc

        return Beta,dBeta

    def _impact_conversion(self,impact_state,params,impact_conditions):

        x,z,phi,vx,vz,dphi = impact_state
        self.Gamma_eff,self.L_eff,self.Forward_Reach,M,Iyy,I_c = self.params
        (BodyContact,Leg1Contact,Leg2Contact) = impact_conditions

        V_tx,V_perp = self.R_WP(np.array([vx,vz]),self.Plane_Angle_rad)

        if Leg1Contact:

            ## CALC BETA ANGLE
            Beta_1 = np.arctan2(np.cos(self.Gamma_eff - phi + self.Plane_Angle_rad), \
                                np.sin(self.Gamma_eff - phi + self.Plane_Angle_rad))
            Beta_1_deg = np.degrees(Beta_1)

            ## CALC DBETA FROM MOMENTUM CONVERSION
            H_V_perp = M*self.L_eff*V_perp*np.cos(Beta_1)
            H_V_tx = M*self.L_eff*V_tx*np.sin(Beta_1)
            H_dphi = Iyy*dphi
            dBeta_1 = 1/(I_c)*(H_V_perp + H_V_tx + H_dphi)

            return Beta_1,dBeta_1

        elif Leg2Contact:

            ## CALC BETA ANGLE
            Beta_2 = np.arctan2( np.cos(self.Gamma_eff + phi - self.Plane_Angle_rad), \
                                -np.sin(self.Gamma_eff + phi - self.Plane_Angle_rad))
            Beta2_deg = np.degrees(Beta_2)


            ## CALC DBETA FROM MOMENTUM CONVERSION
            H_V_perp = M*self.L_eff*V_perp*np.cos(Beta_2)
            H_V_tx = M*self.L_eff*V_tx*np.sin(Beta_2)
            H_dphi = Iyy*dphi
            
            dBeta_2 = 1/(I_c)*(H_V_perp + H_V_tx + H_dphi)

            return Beta_2,dBeta_2

    def _beta_landing(self,impact_leg):

        self.Gamma_eff,self.L_eff,self.Forward_Reach,M,Iyy,I_c = self.params

        if impact_leg == 1:
            return np.pi/2 + self.Gamma_eff

        elif impact_leg == 2:
            return np.pi/2 - self.Gamma_eff
        
    def _beta_prop(self,impact_leg):

        self.Gamma_eff,self.L_eff,self.Forward_Reach,M,Iyy,I_c = self.params

        a = np.sqrt(self.Forward_Reach**2 + self.L_eff**2 - 2*self.Forward_Reach*self.L_eff*np.cos(np.pi/2-self.Gamma_eff))
        Beta = np.arccos((self.L_eff**2 + a**2 - self.Forward_Reach**2)/(2*a*self.L_eff))

        if impact_leg == 1:

            return Beta
        
        elif impact_leg == 2:

            return np.pi-Beta



    def draw_line_dashed(self,surface, color, start_pos, end_pos, width = 1, dash_length = 10, exclude_corners = True):

        # convert tuples to numpy arrays
        start_pos = np.array(start_pos)
        end_pos   = np.array(end_pos)

        # get euclidian distance between start_pos and end_pos
        length = np.linalg.norm(end_pos - start_pos)

        # get amount of pieces that line will be split up in (half of it are amount of dashes)
        dash_amount = int(length / dash_length)

        # x-y-value-pairs of where dashes start (and on next, will end)
        dash_knots = np.array([np.linspace(start_pos[i], end_pos[i], dash_amount) for i in range(2)]).transpose()

        return [pg.draw.line(surface, color, tuple(dash_knots[n]), tuple(dash_knots[n+1]), width)
                for n in range(int(exclude_corners), dash_amount - int(exclude_corners), 2)]

    
if __name__ == "__main__":

    env = SAR_2D_Sim_Interface()
    rospy.spin()

