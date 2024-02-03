#!/usr/bin/env python3
import numpy as np
import os
import rospy

import numpy as np
from gymnasium import spaces

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

class SAR_2D_Base_Interface():

    def __init__(self,Render=True):
        
        ######################
        #   2D ENV CONFIGS
        ######################

        ## INERTIAL PARAMETERS
        self.Ref_Mass = 781.1e-3    # Body Mass [kg]
        self.Ref_Iyy = 5.29e-3      # Body Moment of Inertia [kg*m^2]

        ## GEOMETRIC PARAMETERS
        self.Forward_Reach = 150.0e-3   # [m]
        self.Prop_Front = 72.6e-3 # Prop Distance from Model Origin

        ## EFFECTIVE-GEOEMTRIC PARAMETERS
        self.L_eff = 216.4e-3           # Leg Length [m]
        self.Gamma_eff = 17             # Leg Angle [m]
        self.Lx_eff = self.L_eff*np.sin(np.radians(self.Gamma_eff))
        self.Lz_eff = self.L_eff*np.cos(np.radians(self.Gamma_eff))
        self.Collision_Radius = max(self.L_eff,self.Forward_Reach)


        ## SYSTEM AND FLIGHT PARAMETERS
        self.Thrust_max = 800           # Max thrust per motor [g]
        self.Ang_Acc_max = (9.81*self.Thrust_max*1e-3*self.Prop_Front[0])*2/self.Ref_Iyy
        self.setAngAcc_range([-self.Ang_Acc_max, self.Ang_Acc_max])
        self.I_c = self.Ref_Iyy + self.Ref_Mass*self.L_eff**2

        self.Beta_Min_deg = -(self.Gamma_eff + np.degrees(np.arctan2(self.Forward_Reach-self.Lx_eff,self.Lz_eff)))
        self.Phi_P_B_impact_Min_deg = -self.Beta_Min_deg - self.Gamma_eff + 90
        

        ## PLANE PARAMETERS
        self.Plane_Pos = [1,0]
        self.Plane_Angle_deg = 0
        self.Plane_Angle_rad = np.radians(self.Plane_Angle_deg)

        ## INITIAL LEARNING/REWARD CONFIGS
        self.Trg_Flag = False
        self.MomentCutoff = False
        self.BodyContact_Flag = False
        self.Pad_Connections = 0
        

        ## SPECIAL CONFIGS
        self.state = np.zeros(6)
        self.impact_state = np.full(6,np.nan)



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

        #
        #######################################

        print(f"{GREEN}")
        print("=============================================")
        print("       SAR Base Interface Initialized        ")
        print("=============================================")

        print(f"L_eff: {self.L_eff:.3f} m \t\t Gamma_eff: {self.Gamma_eff:.1f} deg\n")
        print(f"Phi_impact_P_B_Min: {self.Phi_P_B_impact_Min_deg:.1f} deg\n")

        print(f"Thrust_max: {self.Thrust_max:.0f} g")
        print(f"Ang_Acc_Max: {self.Ang_Acc_max:.0f} rad/s^2")

        print(f"{RESET}")

    def setAngAcc_range(self,Ang_Acc_range):
        """Sets the range of allowable angular accelerations for the model

        Args:
            Ang_Acc_range (list): List of min/max angular accelerations [min,max]
        """        

        if max(abs(i) for i in Ang_Acc_range) > self.Ang_Acc_max:
            rospy.logwarn(f"Angular Acceleration range exceeds max value of {self.Ang_Acc_max:.0f} deg/s^2")
            rospy.logwarn(f"Setting Angular Acceleration range to max value")
            self.Ang_Acc_range = [-self.Ang_Acc_max,self.Ang_Acc_max]
        else:
            self.Ang_Acc_range = Ang_Acc_range

    def _getTime(self):

        return self.t
    
    def _get_pose(self):

        gamma_rad,L,PD,M,Iyy,I_c = self.params
        x,z,phi,_,_,_ = self._get_state()

        ## MODEL CoG
        CG = np.array([x,z])

        ## LEG COORDS
        L1 = np.array([ L*np.sin(gamma_rad),-L*np.cos(gamma_rad)])
        L2 = np.array([-L*np.sin(gamma_rad),-L*np.cos(gamma_rad)])

        ## PROP COORDS
        Prop1 = np.array([ PD,0])
        Prop2 = np.array([-PD,0])

        ## CONVERT BODY COORDS TO WORLD COORDS
        L1 = CG + self.R_BW(L1,phi)
        L2 = CG + self.R_BW(L2,phi)
        Prop1 = CG + self.R_BW(Prop1,phi)
        Prop2 = CG + self.R_BW(Prop2,phi)

        return np.array([CG,L1,L2,Prop1,Prop2])

    def _get_impact_conditions(self,x,z,phi):

        impact_flag  = False
        Body_contact = False
        Leg1_contact = False
        Leg2_contact = False

        CG_Pos,Leg1_Pos,Leg2_Pos,Prop1_Pos,Prop2_Pos = self._get_pose()

        ## CHECK FOR CG CONTACT 
        CG_wrt_Plane = self.R_WP((CG_Pos - self.Plane_Pos),self.Plane_Angle_rad)
        if CG_wrt_Plane[1] >= 0:
            impact_flag = True
            Body_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR PROP CONTACT
        for Prop_Pos in [Prop1_Pos,Prop2_Pos]:

            Prop_wrt_Plane = self.R_WP((Prop_Pos - self.Plane_Pos),self.Plane_Angle_rad)
            if Prop_wrt_Plane[1] >= 0:
                impact_flag = True
                Body_contact = True

                return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR LEG1 CONTACT
        Leg1_wrt_Plane = self.R_WP((Leg1_Pos - self.Plane_Pos),self.Plane_Angle_rad)
        if Leg1_wrt_Plane[1] >= 0:
            impact_flag = True
            Leg1_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

        ## CHECK FOR LEG2 CONTACT
        Leg2_wrt_Plane = self.R_WP((Leg2_Pos - self.Plane_Pos),self.Plane_Angle_rad)
        if Leg2_wrt_Plane[1] >= 0:
            impact_flag = True
            Leg2_contact = True

            return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]


        return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

    def _set_state(self,x,z,phi,vx,vz,dphi):

        self.state = (x,z,phi,vx,vz,dphi)

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
        x,z,phi,vx,vz,dphi = self._get_state()

        ## GET CURRENT PARAMS
        gamma_rad,L,PD,M,Iyy,I_c = self.params
        
        ## CREATE BACKGROUND SURFACE
        self.surf = pg.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE_PG)

        ## TRAJECTORY LINE
        r_B_O,V_B_O = self.initial_state
        self.draw_line_dashed(self.surf,GREY_PG,c2p(r_B_O - 5*V_B_O),c2p(r_B_O + 5*V_B_O),width=3)

        ## ORIGIN AXES
        pg.draw.line(self.surf,GREEN_PG,c2p((0,0,0)),c2p((0.1,0)),width=5) # X_w   
        pg.draw.line(self.surf,BLUE_PG, c2p((0,0,0)),c2p((0,0.1)),width=5) # Z_w   
        pg.draw.circle(self.surf,RED_PG,c2p((0,0,0)),radius=4,width=0)


        ## LANDING SURFACE
        pg.draw.line(self.surf,GREY_PG,
                         c2p(self.Plane_Pos + self.R_PW(np.array([-2,0]),self.Plane_Angle_rad)),
                         c2p(self.Plane_Pos + self.R_PW(np.array([+2,0]),self.Plane_Angle_rad)),width=2)
        
        pg.draw.line(self.surf,BLACK_PG,
                         c2p(self.Plane_Pos + self.R_PW(np.array([-0.5,0]),self.Plane_Angle_rad)),
                         c2p(self.Plane_Pos + self.R_PW(np.array([+0.5,0]),self.Plane_Angle_rad)),width=5)
    
        ## LANDING SURFACE AXES
        pg.draw.line(self.surf,GREEN_PG,c2p(self.Plane_Pos),c2p(self.Plane_Pos + self.R_PW(np.array([0.1,0]),self.Plane_Angle_rad)),width=7)  # t_x   
        pg.draw.line(self.surf,BLUE_PG, c2p(self.Plane_Pos),c2p(self.Plane_Pos + self.R_PW(np.array([0,0.1]),self.Plane_Angle_rad)),width=7)  # n_p 
        pg.draw.circle(self.surf,RED_PG,c2p(self.Plane_Pos),radius=4,width=0)


        ## DRAW QUADROTOR
        Pose = self._get_pose()
        pg.draw.line(self.surf,RED_PG,c2p(Pose[0]),c2p(Pose[1]),width=3) # Leg 1
        pg.draw.line(self.surf,BLACK_PG,c2p(Pose[0]),c2p(Pose[2]),width=3) # Leg 2
        pg.draw.line(self.surf,BLACK_PG,c2p(Pose[0]),c2p(Pose[3]),width=3) # Prop 1
        pg.draw.line(self.surf,BLACK_PG,c2p(Pose[0]),c2p(Pose[4]),width=3) # Prop 2
        pg.draw.circle(self.surf,GREY_PG,c2p(Pose[0]),radius=self.Collision_Radius*self.screen_width/self.world_width,width=2)

        ## BODY AXES
        pg.draw.line(self.surf,GREEN_PG,c2p(Pose[0]),c2p(Pose[0] + self.R_BW(np.array([0.05,0]),phi)),width=5)  # B_x   
        pg.draw.line(self.surf,BLUE_PG,c2p(Pose[0]),c2p(Pose[0] + self.R_BW(np.array([0,0.05]),phi)),width=5)  # B_z  

        ## GRAVITY UNIT VECTOR
        g_hat = np.array([0,-1])
        pg.draw.line(self.surf,PURPLE_PG,c2p(Pose[0]),c2p(Pose[0]) + g_hat*25,width=3)

        ## VELOCITY UNIT VECTOR
        v = np.array([vx,vz])
        v_hat = v/np.linalg.norm(v)
        pg.draw.line(self.surf,ORANGE_PG,c2p(Pose[0]),c2p(Pose[0]) + v_hat*25,width=3)




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

        ## STATES TEXT
        text_States = my_font.render(f'States:', True, GREY_PG)
        text_t_step = my_font.render(f'Time Step: {self.t:7.03f} [s]', True, BLACK_PG)
        text_V_mag = my_font.render(f'V_mag: {self.V_mag:.2f} [m/s]', True, BLACK_PG)
        text_Rel_Angle = my_font.render(f'Flight_Angle: {self.V_angle:.2f} [deg]', True, BLACK_PG)

        ## OBSERVATIONS TEXT
        text_Obs = my_font.render(f'Observations:', True, GREY_PG)
        text_Tau = my_font.render(f'Tau: {self._get_obs()[0]:2.2f} [s]', True, BLACK_PG)
        text_theta_x = my_font.render(f'Theta_x: {self._get_obs()[1]:2.2f} [rad/s]', True, BLACK_PG)
        text_D_perp = my_font.render(f'D_perp: {self._get_obs()[2]:2.2f} [m]', True, BLACK_PG)
        text_Plane_Angle = my_font.render(f'Plane Angle: {self.Plane_Angle_deg:3.1f} [deg]', True, BLACK_PG)

        ## ACTIONS TEXT
        text_Actions = my_font.render(f'Actions:', True, GREY_PG)
        text_Trg_Action = my_font.render(f'Trg_Action: {self.action_trg[0]:3.1f}', True, BLACK_PG)
        text_Rot_Action = my_font.render(f'Rot_Action: {self.action_trg[1]:3.1f}', True, BLACK_PG)

        ## OTHER TEXT
        text_Other = my_font.render(f'Other:', True, GREY_PG)
        text_reward = my_font.render(f'Prev Reward: {self.reward:.3f}',True, BLACK_PG)
        text_Tau_trg = my_font.render(f'Tau_trg: {self.Tau_trg:.3f} [s]',True, BLACK_PG)
        text_Tau_CR_trg = my_font.render(f'Tau_CR_trg: {self.Tau_CR_trg:.3f} [s]',True, BLACK_PG)
        text_Tau_CR = my_font.render(f'Tau CR: {self.Tau_CR:.3f} [s]',True, BLACK_PG)
        text_Phi = my_font.render(f'Phi: {np.degrees(phi):.0f} deg',True, BLACK_PG)





        ## DRAW OBJECTS TO SCREEN
        self.screen.blit(self.surf,         (0,0))
        self.screen.blit(text_States,       (5,5))
        self.screen.blit(text_t_step,       (5,5 + 25*1))
        self.screen.blit(text_Rel_Angle,    (5,5 + 25*2))
        self.screen.blit(text_V_mag,        (5,5 + 25*3))

        self.screen.blit(text_Obs,          (5,5 + 25*5))
        self.screen.blit(text_Tau,          (5,5 + 25*6))
        self.screen.blit(text_theta_x,      (5,5 + 25*7))
        self.screen.blit(text_D_perp,       (5,5 + 25*8))
        self.screen.blit(text_Plane_Angle,  (5,5 + 25*9))

        self.screen.blit(text_Actions,      (5,5 + 25*11))
        self.screen.blit(text_Trg_Action,   (5,5 + 25*12))
        self.screen.blit(text_Rot_Action,   (5,5 + 25*13))

        self.screen.blit(text_Other,        (5,5 + 25*15))
        self.screen.blit(text_reward,       (5,5 + 25*16))
        self.screen.blit(text_Tau_trg,      (5,5 + 25*17))
        self.screen.blit(text_Tau_CR_trg,   (5,5 + 25*18))
        self.screen.blit(text_Tau_CR,       (5,5 + 25*19))
        self.screen.blit(text_Phi,          (5,5 + 25*20))






        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(30) # [Hz]
        pg.display.flip()


    def _iterStep(self,n_steps=10):

        for _ in range(n_steps):

            ## UPDATE STATE
            x,z,phi,vx,vz,dphi = self._get_state()

            self.t += self.dt

            z_acc = 0.0
            z = z + self.dt*vz
            vz = vz + self.dt*z_acc

            x_acc = 0.0
            x = x + self.dt*vx
            vx = vx + self.dt*x_acc

            phi_acc = 0.0
            phi = phi + self.dt*dphi
            dphi = dphi + self.dt*phi_acc

            self.state = np.array([x,z,phi,vx,vz,dphi])

    def _iter_step_Rot(self,Rot_action,n_steps=2):

        ## PARAMS
        gamma_rad,L,PD,M,Iyy,I_c = self.params

        ## CURRENT STATE
        x,z,phi,vx,vz,dphi = self._get_state()

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

            self.state = np.array([x,z,phi,vx,vz,dphi])

    def _iter_step_Swing(self, Beta, dBeta, impact_leg, n_steps=2):
        """
        Perform iterative steps for the Swing phase of the SAR environment.

        Args:
            Beta (float): Current angle of the swing leg.
            dBeta (float): Current angular velocity of the swing leg.
            impact_leg (int): Leg number that is impacting the ground.
            n_steps (int, optional): Number of iterative steps to perform. Defaults to 2.

        Returns:
            tuple: Updated angle (Beta) and angular velocity (dBeta) of the swing leg.
        """

        gamma_rad, L, PD, M, Iyy, I_c = self.params

        for _ in range(n_steps):
            if impact_leg == 1:
                ## GRAVITY MOMENT
                M_g = -M * self.g * L * np.cos(Beta) * np.cos(self.Plane_Angle_rad) \
                    + M * self.g * L * np.sin(Beta) * np.sin(self.Plane_Angle_rad)
            if impact_leg == 2:
                ## GRAVITY MOMENT
                M_g = -M * self.g * L * np.cos(Beta) * np.cos(self.Plane_Angle_rad) \
                    + M * self.g * L * np.sin(Beta) * np.sin(self.Plane_Angle_rad)

            ## ITER STEP BETA
            self.t += self.dt
            Beta_acc = M_g / I_c
            Beta = Beta + self.dt * dBeta
            dBeta = dBeta + self.dt * Beta_acc

        return Beta, dBeta
    def _iter_step_Swing(self,Beta,dBeta,impact_leg,n_steps=2):

        gamma_rad,L,PD,M,Iyy,I_c = self.params

        for _ in range(n_steps):

            if impact_leg==1:

                ## GRAVITY MOMENT
                M_g = -M*self.g*L*np.cos(Beta)*np.cos(self.Plane_Angle_rad) \
                        + M*self.g*L*np.sin(Beta)*np.sin(self.Plane_Angle_rad)
                
            if impact_leg==2:

                ## GRAVITY MOMENT
                M_g = -M*self.g*L*np.cos(Beta)*np.cos(self.Plane_Angle_rad) \
                        + M*self.g*L*np.sin(Beta)*np.sin(self.Plane_Angle_rad)
                
                
            ## ITER STEP BETA
            self.t += self.dt
            Beta_acc = M_g/I_c
            Beta = Beta + self.dt*dBeta
            dBeta = dBeta + self.dt*Beta_acc

        return Beta,dBeta

    def _impact_conversion(self,impact_state,params,impact_conditions):

        x,z,phi,vx,vz,dphi = impact_state
        gamma_rad,L,PD,M,Iyy,I_c = self.params
        (BodyContact,Leg1Contact,Leg2Contact) = impact_conditions

        V_tx,V_perp = self.R_WP(np.array([vx,vz]),self.Plane_Angle_rad)

        if Leg1Contact:

            ## CALC BETA ANGLE
            Beta_1 = np.arctan2(np.cos(gamma_rad - phi + self.Plane_Angle_rad), \
                                np.sin(gamma_rad - phi + self.Plane_Angle_rad))
            Beta_1_deg = np.degrees(Beta_1)

            ## CALC DBETA FROM MOMENTUM CONVERSION
            H_V_perp = M*L*V_perp*np.cos(Beta_1)
            H_V_tx = M*L*V_tx*np.sin(Beta_1)
            H_dphi = Iyy*dphi
            dBeta_1 = 1/(I_c)*(H_V_perp + H_V_tx + H_dphi)

            return Beta_1,dBeta_1

        elif Leg2Contact:

            ## CALC BETA ANGLE
            Beta_2 = np.arctan2( np.cos(gamma_rad + phi - self.Plane_Angle_rad), \
                                -np.sin(gamma_rad + phi - self.Plane_Angle_rad))
            Beta2_deg = np.degrees(Beta_2)


            ## CALC DBETA FROM MOMENTUM CONVERSION
            H_V_perp = M*L*V_perp*np.cos(Beta_2)
            H_V_tx = M*L*V_tx*np.sin(Beta_2)
            H_dphi = Iyy*dphi
            
            dBeta_2 = 1/(I_c)*(H_V_perp + H_V_tx + H_dphi)

            return Beta_2,dBeta_2

    def _beta_landing(self,impact_leg):

        gamma_rad,L,PD,M,Iyy,I_c = self.params

        if impact_leg == 1:
            return np.pi/2 + gamma_rad

        elif impact_leg == 2:
            return np.pi/2 - gamma_rad
        
    def _beta_prop(self,impact_leg):

        gamma_rad,L,PD,M,Iyy,I_c = self.params

        a = np.sqrt(PD**2 + L**2 - 2*PD*L*np.cos(np.pi/2-gamma_rad))
        Beta = np.arccos((L**2 + a**2 - PD**2)/(2*a*L))

        if impact_leg == 1:

            return Beta
        
        elif impact_leg == 2:

            return np.pi-Beta



    def R_BW(self,vec,phi):

        R_BW = np.array([
            [ np.cos(phi), np.sin(phi)],
            [-np.sin(phi), np.cos(phi)],
        ])

        return R_BW.dot(vec)
    
    def R_WP(self,vec,theta):

        R_WP = np.array([
            [ np.cos(theta),-np.sin(theta)],
            [ np.sin(theta), np.cos(theta)]
        ])

        return R_WP.dot(vec)
    
    def R_PW(self,vec,theta):

        R_PW = np.array([
            [ np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])

        return R_PW.dot(vec)
    
    def R_PC1(self,vec,Beta1):

        R_PC1 = np.array([
            [ np.cos(Beta1),-np.sin(Beta1)],
            [ np.sin(Beta1), np.cos(Beta1)]
        ])

        return R_PC1.dot(vec)
    
    def R_C1P(self,vec,Beta1):

        R_C1P = np.array([
            [ np.cos(Beta1), np.sin(Beta1)],
            [-np.sin(Beta1), np.cos(Beta1)]
        ])

        return R_C1P.dot(vec)
    
    def R_C1B(self,vec,gamma_rad):

        R_C1B = np.array([
            [ np.sin(gamma_rad), np.cos(gamma_rad)],
            [-np.cos(gamma_rad), np.sin(gamma_rad)],
        ])

        return R_C1B.dot(vec)

    def R_PC2(self,vec,Beta2):

        R_PC2 = np.array([
            [ np.cos(Beta2), np.sin(Beta2)],
            [-np.sin(Beta2), np.cos(Beta2)]
        ])

        return R_PC2.dot(vec)
    
    def R_C2P(self,vec,Beta2):

        R_C2P = np.array([
            [ np.cos(Beta2), np.sin(Beta2)],
            [-np.sin(Beta2), np.cos(Beta2)],
        ])

        return R_C2P.dot(vec)

    def R_C2B(self,vec,gamma_rad):

        R_C2B = np.array([
            [-np.sin(gamma_rad), np.cos(gamma_rad)],
            [-np.cos(gamma_rad),-np.sin(gamma_rad)],
        ])

        return R_C2B.dot(vec)


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

    
