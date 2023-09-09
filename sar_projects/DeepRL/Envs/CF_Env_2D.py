import gymnasium as gym
import numpy as np
from gymnasium import spaces

import pygame
import os

class CF_Env_2D(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {'render.modes': ['human']}

    def __init__(self,My_range=[-8.0,0],Vel_range=[1.5,3.5],Phi_rel_range=[90,180],Tau_0=0.4):
        """
        Args:
            GZ_Timeout (bool, optional): Determines if Gazebo will restart if it freezed. Defaults to False.
            My_range (list, optional): Range of body moment actions (N*mm). Defaults to [0.0,8.0].
            Vel_range (list, optional): Range of flight velocities (m/s). Defaults to [1.5,3.5].
            Phi_range (list, optional): Range of flight angles (Deg). Defaults to [0,90].
            Tau_0 (float, optional): Flight position will start at this Tau value. Defaults to 0.4.
        """   
        gym.Env.__init__(self)

        ## ENV CONFIG SETTINGS
        self.Env_Name = "CF_Env_2D"

        ## TESTING CONDITIONS
        self.Tau_0 = Tau_0          
        self.Vel_range = Vel_range  
        self.Phi_rel_range = Phi_rel_range
        self.My_range = My_range

        ## RESET INITIAL VALUES
        self.K_ep = 0
        self.Trg_threshold = 0.5
        self.D_min = np.inf
        self.Tau_trg = np.inf
        self.Flip_Flag = False
        self.Done = False

        ## DEFINE OBSERVATION SPACE
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
        self.obs_trg = np.zeros(self.observation_space.shape,dtype=np.float32) # Obs values at triggering

        ## DEFINE ACTION SPACE
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.action_trg = np.zeros(self.action_space.shape,dtype=np.float32) # Action values at triggering

        ## PHYSICS PARAMETERS
        self.dt = 0.005  # seconds between state updates
        self.t_step = 0
        self.state = None
        self.start_vals = (0,0)
        self.reward = 0

        ## SIM PARAMETERS
        self.Plane_Pos = [0,0,2.1]
        self.MomentCutoff = False
        self.impact_flag = False
        self.Impact_events = [False,False,False]
        self.BodyContact_flag = False
        self.pad_connections = 0
        self.theta_impact = 0.0
        
        ## SET DIMENSIONAL CONSTRAINTS 
        g = 9.81                # Gravity [m/s^2]
        I_G = 30.46e-6          # Body Moment of Intertia [kg*m^2]
        PD = 0.115/2            # Prop Distance from COM [m]
        e = 0.018               # Distance of leg joint from COM [m]
        L = 0.1                 # Leg Length [m]
        gamma = np.deg2rad(30)  # Leg Angle [m]
        M_G = 0.035             # Body Mass [kg]
        M_L = 0.002             # Leg Mass [kg]
        self.params = (L,e,gamma,M_G,M_L,g,PD,I_G)

        ## ENV LIMITS
        self.world_width = 4.0  # [m]
        self.world_height = 3.0 # [m]
        self.t_threshold = 600


        ## RENDERING PARAMETERS
        self.RENDER = False
        self.screen_width = 1000
        self.screen_height = self.screen_width*self.world_height/self.world_width
        self.screen = None
        self.clock = None
        self.isopen = True

    def reset(self, seed=None, options=None, Vel=None, Phi=None):


        ## RESET RECORDED VALUES
        self.Done = False
        self.D_min = np.inf       # Reset max distance from landing surface [m]
        self.Tau_trg = np.inf     # Reset Tau triggering value [s]
        self.obs_trg = np.zeros_like(self.observation_space.high)
        self.action_trg = np.zeros_like(self.action_space.high)

        ## RESET LOGGING CONDITIONS 
        self.K_ep += 1

        ## RESET PHYSICS PARAMS
        self.t_step = 0
        self.Flip_Flag = False
        self.MomentCutoff = False
        self.impact_flag = False
        self.Impact_events = [False,False,False]
        self.BodyContact_flag = False
        self.pad_connections = 0
        self.theta_impact = 0.0

        ## RESET POSITION RELATIVE TO LANDING SURFACE (BASED ON STARTING TAU VALUE)
        # (Derivation: Research_Notes_Book_3.pdf (6/22/23))

        r_PO = np.array(self.Plane_Pos)  # Plane Position w/r to origin

        n_hat = np.array([0,0,1])   # Plane normal vector
        t_x = np.array([1,0,0])     # Plane normal vector
        t_y = np.array([0,1,0])     # Plane normal vector


        ## SAMPLE VELOCITY AND FLIGHT ANGLE
        if Vel == None or Phi == None:
            Vel,Phi = self._sample_flight_conditions()

        else:
            Vel = Vel   # Flight velocity
            Phi = Phi   # Flight angle  

        ## CALCULATE GLOABAL VEL VECTORS
        V_x = Vel*np.cos(np.deg2rad(Phi))
        V_y = 0
        V_z = Vel*np.sin(np.deg2rad(Phi))
        
        V_BO = np.array([V_x,V_y,V_z])      # Flight Velocity
        V_hat = V_BO/np.linalg.norm(V_BO)   # Flight Velocity unit vector

        ## RELATIVE VEL VECTORS
        V_perp = V_BO.dot(n_hat)
        V_tx = V_BO.dot(t_x)
        V_ty = V_BO.dot(t_y)

        ## CALC STARTING/VELOCITY LAUCH POSITION            
        if V_hat.dot(n_hat) <= 0.35:  # Velocity near parallel to landing surface

            ## ## MINIMUM DISTANCE TO START POLICY TRAINING
            D_perp = 0.2  # Ensure a reasonable minimum perp distance [m]

            ## INITIAL POSITION RELATIVE TO PLANE
            r_BP = (D_perp/(V_hat.dot(n_hat)))*V_hat

            ## INITIAL POSITION IN GLOBAL COORDS
            r_BO = r_PO - r_BP 

        else: # Velocity NOT parallel to surface

            ## CALC STARTING DISTANCE WHERE POLICY IS MONITORED
            D_perp = (self.Tau_0*V_perp)    # Initial perp distance
            D_perp = max(D_perp,0.2)        # Ensure a reasonable minimum distance [m]

            ## INITIAL POSITION RELATIVE TO PLANE
            r_BP = (D_perp/(V_hat.dot(n_hat)))*V_hat

            ## INITIAL POSITION IN GLOBAL COORDS
            r_BO = r_PO - r_BP 

            
        ## SET INITIAL STATE FOR THE SYSTEM
        z_0 = r_BO[2]
        x_0 = 0.0
        theta = 0.0
        dtheta = 0.0
        self.state = (x_0,V_x,z_0,V_z,theta,dtheta)

        self.start_vals = (Vel,Phi)

        return self._get_obs(), {}

    def _iter_step(self):

        ## UPDATE STATE
        x,vx,z,vz,theta,dtheta = self._get_state()

        self.t_step += 1

        z_acc = 0.0
        z = z + self.dt*vz
        vz = vz + self.dt*z_acc

        x_acc = 0.0
        x = x + self.dt*vx
        vx = vx + self.dt*x_acc

        theta_acc = 0.0
        theta = theta + self.dt*dtheta
        dtheta = dtheta + self.dt*theta_acc

        self.state = (x,vx,z,vz,theta,dtheta)

    def _get_state(self):

        return self.state

    def _get_obs(self):

        x,vx,z,vz,theta,dtheta = self._get_state()

        ## UPDATE OBSERVATION
        D_perp = self.Plane_Pos[2] - z
        Tau = D_perp/vz + 1e-6
        Theta_x = vx/(D_perp + 1e-6)

        return np.array([Tau,Theta_x,D_perp],dtype=np.float32)
    
    def step(self, action):
        
        if self.RENDER:
            self.render()

        ########## PRE-FLIP TRIGGER ##########
        if action[0] < self.Trg_threshold:

            ## GRAB CURRENT OBSERVATION
            obs = self._get_obs()

            ## CHECK FOR IMPACT
            x,vx,z,vz,theta,dtheta = self._get_state()
            self.impact_flag,self.Impact_events = self._impact_conditions(x,z,theta)

            ## CHECK FOR DONE
            self.Done = bool(
                self.Done
                or (self.impact_flag or self.BodyContact_flag)  # BODY CONTACT W/O FLIP TRIGGER
            )

            ## UPDATE MINIMUM DISTANCE
            D_perp = obs[2]
            if not self.Done:
                if D_perp <= self.D_min:
                    self.D_min = D_perp 


            terminated = self.Done
            truncated = bool(self.t_step >= self.t_threshold) # EPISODE TIMEOUT (SIM TIME)

            ## CALCULATE REWARD
            reward = 0

            ## UPDATE SIM 
            self._iter_step()

            reward = 0
            
            ## UDPATE OBSERVATION
            obs = self._get_obs()
                
        ########## POST-FLIP TRIGGER ##########
        elif action[0] > self.Trg_threshold:

            ## GRAB CURRENT OBSERVATION
            obs = self._get_obs()   # Return this observation because reward and future 
                                    # are defined by action taken here. Not obs at end of episode.

            ## SAVE TRIGGERING OBSERVATION AND ACTIONS
            self.obs_trg = obs
            self.Tau_trg = obs[0]
            self.action_trg = action
            self.Flip_Flag = True


            ## COMPLETE REST OF SIMULATION
            self._finish_sim(action)

            terminated = self.Done = True
            truncated = False
            
            ## CALCULATE REWARD
            reward = self._CalcReward()
            self.reward = reward

        return (
            obs,
            reward,
            terminated,
            truncated,
            {},

        )
    
    def _finish_sim(self,action):

        (L,e,gamma,M_G,M_L,g,PD,I_G) = self.params
        
        ## Scale Action
        scaled_action = 0.5 * (action[1] + 1) * (self.My_range[1] - self.My_range[0]) + self.My_range[0]

        while not self.Done:

            self.t_step += 1
            x,vx,z,vz,theta,dtheta = self._get_state()

            ## CHECK IF PAST 90 DEG
            if np.abs(theta) < np.deg2rad(90) and self.MomentCutoff == False:

                ## CONVERT ACTION RANGE TO MOMENT RANGE
                My = scaled_action*1e-3

            ## TURN OFF BODY MOMENT IF PAST 90 DEG
            else: 
                self.MomentCutoff= True
                My = 0

            ## BODY MOMENT/PROJECTILE FLIGHT
            if self.impact_flag == False:

                ## UPDATE STATE
                z_acc = -My/(M_G*PD)*np.sin(np.pi/2-theta) - g
                z = z + self.dt*vz
                vz = vz + self.dt*z_acc

                x_acc = -My/(M_G*PD)*np.cos(np.pi/2-theta)
                x = x + self.dt*vx
                vx = vx + self.dt*x_acc

                theta_acc = My/I_G
                theta = theta + self.dt*dtheta
                dtheta = dtheta + self.dt*theta_acc

                self.state = (x,vx,z,vz,theta,dtheta)

                ## CHECK FOR IMPACT
                self.impact_flag,self.Impact_events = self._impact_conditions(x,z,theta)

                ## CHECK FOR DONE
                self.Done = bool(
                    self.Done
                    or self.t_step >= self.t_threshold
                    or (vz <= -5.0 and self._get_obs()[2] >= 0.5) # IF SAR IS FALLING
                )
                
            elif self.impact_flag == True:

                self.theta_impact = np.rad2deg(theta)

                ## BODY CONTACT
                if self.Impact_events[0] == True:
                    self.BodyContact_flag = True
                    self.pad_connections = 0
                    self.Done = True

                ## LEG 1 CONTACT
                elif self.Impact_events[1] == True:
                    
                    beta_0,dbeta_0 = self._impact_conversion(self.state,self.Impact_events)
                    beta = beta_0
                    dbeta = dbeta_0

                    l = L/2 # Half leg length [m]
                    I_C = M_G*(4*l**2 + e**2 + 4*l*e*np.sin(gamma)) + I_G   # Inertia about contact point

                    ## FIND IMPACT COORDS
                    r_G_C1_0 = np.array([
                        [-(L+e*np.sin(gamma))*np.cos(beta_0)+e*np.cos(gamma)*np.sin(beta_0)],
                        [-(L+e*np.sin(gamma))*np.sin(beta_0)-e*np.cos(gamma)*np.cos(beta_0)]])

                    r_O_G = np.array([[x],[z]])
                    r_O_C1 = r_O_G - r_G_C1_0

                    ## SOLVE SWING ODE
                    while not self.Done:
                        
                        self.t_step += 1

                        beta_acc = M_G*g/I_C*(2*l*np.cos(beta) - e*np.sin(beta-gamma))
                        beta = beta + self.dt*dbeta
                        dbeta = dbeta + self.dt*beta_acc

                        if beta > self._beta_landing(impact_leg=1):
                            self.BodyContact_flag = False
                            self.pad_connections = 4
                            self.Done = True

                        elif beta < self._beta_prop(impact_leg=1):
                            self.BodyContact_flag = True
                            self.pad_connections = 2
                            self.Done = True
                        elif self.t_step >= self.t_threshold:
                            self.BodyContact_flag = False
                            self.pad_connections = 2
                            self.Done = True

                        ## SOLVE FOR SWING BEHAVIOR IN GLOBAL COORDINATES
                        r_G_C1 = np.array([
                            [-(L+e*np.sin(gamma))*np.cos(beta)+e*np.cos(gamma)*np.sin(beta)],
                            [-(L+e*np.sin(gamma))*np.sin(beta)-e*np.cos(gamma)*np.cos(beta)]])

                        theta = -((np.pi/2-gamma) + beta) # Convert beta in (e^) to theta in (G^)
                        x = r_O_C1[0,0] + r_G_C1[0,0]  # x
                        z = r_O_C1[1,0] + r_G_C1[1,0]  # z


                        self.state = (x,0,z,0,theta,0)

                        if self.RENDER:
                            self.render()

                ## LEG 2 CONTACT
                elif self.Impact_events[2] == True:

                    beta_0,dbeta_0 = self._impact_conversion(self.state,self.Impact_events)
                    beta = beta_0
                    dbeta = dbeta_0

                    l = L/2 # Half leg length [m]
                    I_C = M_G*(4*l**2 + e**2 + 4*l*e*np.sin(gamma)) + I_G   # Inertia about contact point

                    ## FIND IMPACT COORDS
                    r_G_C2_0 = np.array(
                                [[-(L+e*np.sin(gamma))*np.cos(beta_0)-e*np.cos(gamma)*np.sin(beta_0)],
                                [-(L+e*np.sin(gamma))*np.sin(beta_0)+e*np.cos(gamma)*np.cos(beta_0)]])

                    r_O_G = np.array([[x],[z]])
                    r_O_C2 = r_O_G - r_G_C2_0

                    ## SOLVE SWING ODE
                    while not self.Done:

                        self.t_step += 1

                        beta_acc = M_G*g/I_C*(2*l*np.cos(beta) + e*np.sin(beta+gamma))
                        beta = beta + self.dt*dbeta
                        dbeta = dbeta + self.dt*beta_acc

                        if beta < self._beta_landing(impact_leg=2):
                            self.BodyContact_flag = False
                            self.pad_connections = 4
                            self.Done = True

                        elif beta > self._beta_prop(impact_leg=2):
                            self.BodyContact_flag = True
                            self.pad_connections = 2
                            self.Done = True

                        elif self.t_step >= self.t_threshold:
                            self.BodyContact_flag = False
                            self.pad_connections = 2
                            self.Done = True

                        ## SOLVE FOR SWING BEHAVIOR IN GLOBAL COORDINATES
                        r_G_C2 = np.array(
                        [[-(L+e*np.sin(gamma))*np.cos(beta)-e*np.cos(gamma)*np.sin(beta)],
                        [-(L+e*np.sin(gamma))*np.sin(beta)+e*np.cos(gamma)*np.cos(beta)]])

                        theta = -((np.pi/2+gamma) + beta) # Convert beta in (e^) to theta in (G^)
                        x = r_O_C2[0,0] + r_G_C2[0,0]  # x
                        z = r_O_C2[1,0] + r_G_C2[1,0]  # z


                        self.state = (x,0,z,0,theta,0)

                        if self.RENDER:
                            self.render()


            ## UPDATE MINIMUM DISTANCE
            D_perp = self._get_obs()[2]
            if not self.Done:
                if D_perp <= self.D_min:
                    self.D_min = D_perp 


            if self.RENDER:
                self.render()
        
    def _sample_flight_conditions(self):
        """This function samples the flight velocity and angle from the supplied range.
        Velocity is sampled from a uniform distribution. Phi is sampled from a set of 
        uniform distributions which are weighted such that edge cases are only sampled 10% of the time.
        Poor performance on edge cases can cause poor learning convergence.

        Returns:
            vel,phi: Sampled flight velocity and flight angle
        """        

        ## SAMPLE VEL FROM UNIFORM DISTRIBUTION IN VELOCITY RANGE
        Vel_Low = self.Vel_range[0]
        Vel_High = self.Vel_range[1]
        Vel = np.random.uniform(low=Vel_Low,high=Vel_High)

        ## SAMPLE RELATIVE PHI FROM A WEIGHTED SET OF UNIFORM DISTRIBUTIONS
        Phi_rel_Low = self.Phi_rel_range[0]
        Phi_rel_High = self.Phi_rel_range[1]
        Phi_rel_Range = Phi_rel_High-Phi_rel_Low

        Dist_Num = np.random.choice([0,1,2],p=[0.05,0.9,0.05]) # Probability of sampling distribution

        if Dist_Num == 0: # Low Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_Low, high=Phi_rel_Low + 0.1*Phi_rel_Range)
        elif Dist_Num == 1: # Medium Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_Low + 0.1*Phi_rel_Range, high=Phi_rel_High - 0.1*Phi_rel_Range)
        elif Dist_Num == 2: # High Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_High - 0.1*Phi_rel_Range, high=Phi_rel_High)

        ## CONVERT RELATIVE PHI TO GLOBAL PHI
        Phi_global = (180 + Phi_rel) - 180 # (Derivation: Research_Notes_Book_3.pdf (7/5/23))
        
        return Vel,Phi_global

    def render(self):

        ## SET DEFAULT WINDOW POSITION
        z = 500
        y = 700
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (z,y)
        
        ## CONVERT COORDINATES TO PIXEL LOCATION
        def c2p(Pos):

            x_offset = 2    # [m]
            y_offset = 0.3  # [m]

            scale_x = self.screen_width/self.world_width
            scale_y = self.screen_height/self.world_height
            
            x_p = (x_offset+Pos[0])*scale_x # [pixels]
            y_p = (y_offset+Pos[1])*scale_y # [pixels]
            
            return (x_p,y_p)


        ## DEFINE COLORS
        WHITE = (255,255,255)
        BLACK = (0,0,0)
        BLUE = (29,123,243)
        RED = (255,0,0)

        ## INITIATE SCREEN AND CLOCK ON FIRST LOADING
        if self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_width,self.screen_height))
        
        if self.clock is None:
            self.clock = pygame.time.Clock()

        if self.state is None:
            return None
        x,vx,z,vz,theta,dtheta = self._get_state()

        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE)

        ## ENVIRONMENT LINES
        pygame.draw.line(self.surf,BLACK,c2p((0,-5)),c2p((0,5)),width=1) # Z-axis       
        pygame.draw.line(self.surf,BLACK,c2p((-5,self.Plane_Pos[2])),c2p((5,self.Plane_Pos[2])),width=2) # Ceiling Line
        pygame.draw.line(self.surf,BLACK,c2p((-5,0)),c2p((5,0)),width=2) #  Ground Line

        

        ## CREATE QUADROTOR
        Pose = self._get_pose(x,z,theta)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[1]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[2]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[3]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[4]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[5]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[6]),width=3)

        ## FLIP TRIGGER INDICATOR
        if self.Flip_Flag == True:
            pygame.draw.circle(self.surf,BLACK,c2p((x,z)),radius=7,width=3)
            pygame.draw.circle(self.surf,RED,c2p((x,z)),radius=4,width=0)
        else:
            pygame.draw.circle(self.surf,BLACK,c2p((x,z)),radius=7,width=3)
            pygame.draw.circle(self.surf,BLUE,c2p((x,z)),radius=4,width=0)
    
        

        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pygame.transform.flip(self.surf, False, True)

        Vel = self.start_vals[0]
        phi = self.start_vals[1]

        ## WINDOW TEXT
        my_font = pygame.font.SysFont(None, 30)
        text_t_step = my_font.render(f'Time Step: {self.t_step:03d}', True, BLACK)
        text_Vz = my_font.render(f'Vz: {vz:.2f}', True, BLACK)
        text_Vel = my_font.render(f'Vel: {Vel:.2f}  Phi: {phi:.2f}', True, BLACK)
        text_Tau = my_font.render(f'Tau: {self._get_obs()[0]:.3f}', True, BLACK)
        text_dTau = my_font.render(f'dTau: {0.0:.3f}', True, BLACK)
        text_z_acc = my_font.render(f'z_acc: {0.0:.3f}', True, BLACK)
        text_reward = my_font.render(f'Prev Reward: {self.reward:.3f}',True, BLACK)

        ## WINDOW TEXT LOCATIONS
        self.screen.blit(self.surf,     (0,0))
        self.screen.blit(text_t_step,   (5,5))
        self.screen.blit(text_Vel,      (5,30))
        self.screen.blit(text_Vz,       (5,55))
        self.screen.blit(text_Tau,      (5,80))
        self.screen.blit(text_dTau,     (5,105))
        self.screen.blit(text_z_acc,    (5,130))
        self.screen.blit(text_reward,   (5,155))


        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(60) # [Hz]
        pygame.display.flip()

    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False

    def _CalcReward(self):

        ## DISTANCE REWARD 
        R_dist = np.clip(1/np.abs(self.D_min + 1e-6),0,15)/15
        
        ## TAU TRIGGER REWARD
        R_tau = np.clip(1/np.abs(self.Tau_trg - 0.2),0,15)/15

        ## IMPACT ANGLE REWARD
        Beta_global = -self.theta_impact
        Phi_Plane = 180
        Beta_rel = -180 + Beta_global + Phi_Plane
        R_angle = 0.5*(-np.cos(np.deg2rad(Beta_rel)) + 1) # (Derivation: Research_Notes_Book_3.pdf (6/21/23))

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
    
    def _impact_conditions(self,x_pos,z_pos,theta):
        
        impact_flag  = False
        Body_contact = False
        Leg1_contact = False
        Leg2_contact = False

        MO_z,_,_,Leg1_z,Leg2_z,Prop1_z,Prop2_z = self._get_pose(x_pos,z_pos,theta)[:,1]

        if any(x >= self.Plane_Pos[2] for x in [MO_z,Prop1_z,Prop2_z]):
            impact_flag = True
            Body_contact = True
            
        if Leg1_z >= self.Plane_Pos[2]:
            impact_flag = True
            Leg1_contact = True

        if Leg2_z >= self.Plane_Pos[2]:
            impact_flag = True
            Leg2_contact = True

        return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

    def _get_pose(self,x_pos,z_pos,theta):           
        """Returns position data of all model lines for a given state

        Args:
            model_state (list): [x,z,theta]

        Returns:
            model_pose (tuple): Returns a tuple of all line endpoint coordinates
        """        

        (L,e,gamma,M_G,M_L,g,PD,I_G) = self.params


        ## DEFINE LOCATION OF MODEL ORIGIN
        CG = np.array([x_pos,z_pos])


        ## UPDATE LINE COORDINATES ( These are in body (bx^,bz^) coordinates from MO)
        P1 = np.array([ e,0]) # Hinge_1 Location
        P2 = np.array([-e,0])
        
        L1 = np.array([e + L*np.sin(gamma), # Leg_1 location
                          -L*np.cos(gamma)])
        L2 = np.array([-e - L*np.sin(gamma),
                           -L*np.cos(gamma)])

        Prop1 = np.array([ PD, 0]) # Prop_1 Location
        Prop2 = np.array([-PD, 0])

            
        ## CREATE ROTATION MATRIX FROM THETA VALUE
        R = np.array([[np.cos(-theta),-np.sin(-theta)],
                        [np.sin(-theta), np.cos(-theta)]])

        ## TRANSLATE AND ROTATE BODY COORDS INTO GLOBAL (bx^ -> ex^)
        P1 = CG + R.dot(P1)
        P2 = CG + R.dot(P2)
        L1 = CG + R.dot(L1)
        L2 = CG + R.dot(L2)
        Prop1 = CG + R.dot(Prop1)
        Prop2 = CG + R.dot(Prop2)

        return np.array([CG,P1,P2,L1,L2,Prop1,Prop2])

    def _impact_conversion(self,Impact_state,Impact_events):
        """Converts impact conditions to rotational initial conditions

        Args:
            impact_cond (list, optional): Impact conditions [x,Vx,z,Vz,theta,dtheta]. Defaults to [0,0,0,0].
            gamma (float, optional): Leg angle. Defaults to None.
            L (float, optional): Leg angle. Defaults to None.

        Returns:
            beta,dbeta: Rotational initial conditions
        """        

        (L,e,gamma,M_G,M_L,G,PD,I_G) = self.params

        l = L/2
        I_c = M_G*(4*l**2 + e**2 + 4*l*e*np.sin(gamma)) + I_G

        x,vx,z,vz,theta,dtheta = Impact_state

        ## SWAP THETA SIGNS TO PRESERVE MATH WORK (SDOF_Analytical_Model.pdf)
        # Convert from G_y^ to e_y^ coordinate system
        theta = -theta
        dtheta = -dtheta

        if Impact_events[1] == True:

            beta_0 = theta - (np.pi/2 - gamma)

            H_dtheta = I_G*dtheta                                       # Angular Momentum from dtheta
            H_vx = -M_G*vx*(2*l*np.cos(gamma+theta)-e*np.sin(theta))    # Angular momentum from vel_x
            H_vy =  M_G*vz*(2*l*np.sin(gamma+theta)+e*np.cos(theta))    # Angular momentum from vel_y

        elif Impact_events[2] == True:

            beta_0 = theta - (np.pi/2 + gamma)

            H_dtheta = I_G*dtheta                                       # Angular Momentum from dtheta
            H_vx = -M_G*vx*(2*l*np.cos(gamma-theta)+e*np.sin(theta))    # Angular momentum from vel_x
            H_vy =  M_G*vz*(2*l*np.sin(gamma-theta)+e*np.cos(theta))    # Angular momentum from vel_y


        dbeta_0 = 1/I_c*(H_dtheta + H_vx + H_vy)


        return beta_0,dbeta_0

    def _beta_prop(self,impact_leg=1):
        """Returns minimum beta angle for when propellar contact occurs

        Args:
            gamma (float, optional): Optional argument for specific leg angle. Defaults to class value.
            L (float, optional): Optional argument for specific leg length. Defaults to class value.

        Returns:
            beta_prop (float): Minimum valid beta angle at which propellars hit ceiling
        """        

        (L,e,gamma,M_G,M_L,g,PD,I_G) = self.params

        a = np.sqrt((PD-e)**2+L**2-2*(PD-e)*L*np.cos(np.pi/2-gamma))
        beta_prop = np.arccos((a**2+L**2-(PD-e)**2)/(2*a*L))

        if impact_leg == 1:
            return beta_prop

        elif impact_leg == 2:
            return np.pi - beta_prop

    def _beta_landing(self,impact_leg=1):
        """Returns the max beta value for a given gamma and model parameters

        Args:
            gamma (float, optional): Optional argument for specific leg angle. Defaults to class value.
            L (float, optional): Optional argument for specific leg length. Defaults to class value.

        Returns:
            beta_contact: Max beta angle
        """        

        (L,e,gamma,M_G,M_L,g,PD,I_G) = self.params


        beta_contact = np.pi/2 + gamma

        if impact_leg == 1:
            return beta_contact

        elif impact_leg == 2:
            return np.pi - beta_contact


if __name__ == '__main__':
    env = CF_Env_2D(Vel_range=[1.0,3.0],Phi_rel_range=[120,135])
    env.RENDER = True

    for ep in range(25):
        Vel = 2.5
        Phi = 40
        obs,_ = env.reset(Vel=Vel,Phi=Phi)

        Done = False
        while not Done:

            action = env.action_space.sample()
            # action = np.zeros_like(action)
            obs,reward,Done,truncated,_ = env.step(action)

        print(f"Episode: {ep} \t Reward: {reward:.3f}")

    env.close()



                