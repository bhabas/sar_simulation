import gymnasium as gym
import numpy as np
from gymnasium import spaces

import pygame
import os

from stable_baselines3.common.env_checker import check_env




class CF_Env_2D(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {'render.modes': ['human']}

    def __init__(self,My_range=[0.0,8.0],Vel_range=[1.5,3.5],Phi_range=[0,90],Tau_0=0.4):
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
        self.Phi_range = Phi_range
        self.My_range = My_range

        ## RESET INITIAL VALUES
        self.K_ep = 0
        self.Flip_threshold = 0.5
        self.D_min = 50.0
        self.Tau_trg = 50.0
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
        self.obs = None
        self.start_vals = (0,0)

        ## SIM PARAMETERS
        self.h_ceil = 2.1       # Ceiling Height [m]
        self.MomentCutoff = False
        self.Impact_flag = False
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
        D_perp = self.h_ceil - z
        Tau = D_perp/vz
        Theta_x = vx/(D_perp+1e-3)

        return np.array([Tau,Theta_x,D_perp],dtype=np.float32)
    
    def step(self, action):
        
        self._iter_step()

        ## BASIC FLIGHT   
        if action[0] < self.Flip_threshold:

            
            self.obs = self._get_obs()
            x,vx,z,vz,theta,dtheta = self._get_state()
            Tau,Theta_x,D_perp = self._get_obs()


            ## CHECK FOR IMPACT
            self.Impact_flag,self.Impact_events = self.impact_conditions(x,z,theta)

            ## CHECK FOR DONE
            terminated = bool(
                self.t_step >= self.t_threshold
                or z < 0.2
                or self.Impact_flag
            )

            if not terminated:

                if D_perp <= self.d_min:
                    self.d_min = D_perp

            reward = 0
                
        ## EXECUTE FLIP MANEUVER
        elif action[0] > self.Flip_threshold:

            self.Flip_Flag = True
            self.Tau_trg = 0.25
            reward = self.finish_sim(action) # Reward corresponds directly to this (State,Action) pair not future ones
            terminated = True
        
        truncated = False
        return np.array(self.obs,dtype=np.float32), reward, terminated, truncated, {}
    
    def finish_sim(self,action):

        done = False
        (L,e,gamma,M_G,M_L,g,PD,I_G) = self.params
        
        ## Scale Action
        scaled_action = 0.5 * (action[1] + 1) * (self.My_range[1] - self.My_range[0]) + self.My_range[0]

        while not done:

            self.t_step += 1
            x,vx,z,vz,theta,dtheta = self._get_state()

            ## CHECK IF PAST 90 DEG
            if np.abs(theta) < np.deg2rad(90) and self.MomentCutoff == False:

                ## CONVERT ACTION RANGE TO MOMENT RANGE
                My = -scaled_action*1e-3

            ## TURN OFF BODY MOMENT IF PAST 90 DEG
            else: 
                self.MomentCutoff= True
                My = 0

            ## BODY MOMENT/PROJECTILE FLIGHT
            if self.Impact_flag == False:

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

                ## UPDATE OBSERVATION
                d_ceil = self.h_ceil - z 
                Tau = d_ceil/vz
                Theta_x = -vx/(d_ceil + 1e-3)

                ## CHECK FOR IMPACT
                self.Impact_flag,self.Impact_events = self.impact_conditions(x,z,theta)

                ## CHECK FOR DONE
                done = bool(
                    self.t_step >= self.t_threshold
                    or z < 0.2
                )

                if not done:

                    if d_ceil <= self.d_min:
                        self.d_min = d_ceil
                
            elif self.Impact_flag == True:

                self.theta_impact = np.rad2deg(theta)

                ## BODY CONTACT
                if self.Impact_events[0] == True:
                    self.BodyContact_flag = True
                    self.pad_connections = 0
                    done = True

                ## LEG 1 CONTACT
                elif self.Impact_events[1] == True:
                    
                    beta_0,dbeta_0 = self.impact_conversion(self.state,self.Impact_events)
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
                    while not done:
                        
                        self.t_step += 1

                        beta_acc = M_G*g/I_C*(2*l*np.cos(beta) - e*np.sin(beta-gamma))
                        beta = beta + self.dt*dbeta
                        dbeta = dbeta + self.dt*beta_acc

                        if beta > self.beta_landing(impact_leg=1):
                            self.BodyContact_flag = False
                            self.pad_connections = 4
                            done = True

                        elif beta < self.beta_prop(impact_leg=1):
                            self.BodyContact_flag = True
                            self.pad_connections = 2
                            done = True
                        elif self.t_step >= self.t_threshold:
                            self.BodyContact_flag = False
                            self.pad_connections = 2
                            done = True

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

                    beta_0,dbeta_0 = self.impact_conversion(self.state,self.Impact_events)
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
                    while not done:

                        self.t_step += 1

                        beta_acc = M_G*g/I_C*(2*l*np.cos(beta) + e*np.sin(beta+gamma))
                        beta = beta + self.dt*dbeta
                        dbeta = dbeta + self.dt*beta_acc

                        if beta < self.beta_landing(impact_leg=2):
                            self.BodyContact_flag = False
                            self.pad_connections = 4
                            done = True

                        elif beta > self.beta_prop(impact_leg=2):
                            self.BodyContact_flag = True
                            self.pad_connections = 2
                            done = True

                        elif self.t_step >= self.t_threshold:
                            self.BodyContact_flag = False
                            self.pad_connections = 2
                            done = True

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


            if self.RENDER:
                self.render()
        
        self.reward = self.CalcReward()
        return self.reward


    def reset(self, seed=None, options=None):

        self.K_ep += 1

        ## RESET PHYSICS PARAMS
        self.t_step = 0
        self.Flip_Flag = False
        self.MomentCutoff = False
        self.Impact_flag = False
        self.Impact_events = [False,False,False]
        self.BodyContact_flag = False
        self.pad_connections = 0
        self.theta_impact = 0.0
        self.Tau_trg = 500
        self.d_min = 500
        
        ## RESET STATE
        vel = np.random.uniform(low=1.5,high=3.5)
        phi = np.random.uniform(low=30,high=90)

        # vel = 3.0
        # phi = 70

        vx_0 = vel*np.cos(np.deg2rad(phi))
        vz_0 = vel*np.sin(np.deg2rad(phi))

        ## RESET OBSERVATION
        Tau_0 = 0.5
        D_perp_0 = Tau_0*vz_0+1e-3
        Theta_x = -vx_0/D_perp_0
        self.obs = (Tau_0,Theta_x,D_perp_0)


        z_0 = self.h_ceil - D_perp_0
        x_0 = 0.0

        theta = 0.0
        dtheta = 0.0
        self.state = (x_0,vx_0,z_0,vz_0,theta,dtheta)
        self.start_vals = (vel,phi)

        return np.array(self.obs,dtype=np.float32), {}

    def render(self):

        ## SET DEFAULT WINDOW POSITION
        z = 500
        y = 700
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (z,y)
        
        ## CONVERT COORDINATES TO PIXEL LOCATION
        def c2p(Pos):

            x_offset = 1    # [m]
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
        pygame.draw.line(self.surf,BLACK,c2p((-5,self.h_ceil)),c2p((5,self.h_ceil)),width=2) # Ceiling Line
        pygame.draw.line(self.surf,BLACK,c2p((-5,0)),c2p((5,0)),width=2) #  Ground Line

        

        ## CREATE QUADROTOR
        Pose = self.get_pose(x,z,theta)
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
        # text_Vz = my_font.render(f'Vz: {vz:.2f}', True, BLACK)
        # text_Vel = my_font.render(f'Vel: {Vel:.2f}  Phi: {phi:.2f}', True, BLACK)
        # text_Tau = my_font.render(f'Tau: {self.obs[0]:.3f}', True, BLACK)
        # text_dTau = my_font.render(f'dTau: {0.0:.3f}', True, BLACK)
        # text_z_acc = my_font.render(f'z_acc: {0.0:.3f}', True, BLACK)
        # text_reward = my_font.render(f'Prev Reward: {self.reward:.3f}',True, BLACK)

        ## WINDOW TEXT LOCATIONS
        self.screen.blit(self.surf,     (0,0))
        self.screen.blit(text_t_step,   (5,5))
        # self.screen.blit(text_Vel,      (5,30))
        # self.screen.blit(text_Vz,       (5,55))
        # self.screen.blit(text_Tau,      (5,80))
        # self.screen.blit(text_dTau,     (5,105))
        # self.screen.blit(text_z_acc,    (5,130))
        # self.screen.blit(text_reward,   (5,155))


        ## WINDOW/SIM UPDATE RATE
        self.clock.tick(60) # [Hz]
        pygame.display.flip()

    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False

    def CalcReward(self):

        R0 = np.clip(1/np.abs(self.Tau_trg-0.18),0,20)/20
        R0 *= 0.05

        ## DISTANCE REWARD 
        R1 = np.clip(1/np.abs(self.d_min),0,10)/10
        R1 *= 0.1

        ## IMPACT ANGLE REWARD
        R2 = np.clip(np.abs(self.theta_impact)/120,0,1)
        R2 *= 0.2

        ## PAD CONTACT REWARD
        if self.pad_connections >= 3: 
            if self.BodyContact_flag == False:
                R3 = 0.65
            else:
                R3 = 0.4
        elif self.pad_connections == 2: 
            if self.BodyContact_flag == False:
                R3 = 0.2
            else:
                R3 = 0.1
        else:
            R3 = 0.0

        return R0 + R1 + R2 + R3
    
    def impact_conditions(self,x_pos,z_pos,theta):
        
        impact_flag  = False
        Body_contact = False
        Leg1_contact = False
        Leg2_contact = False

        MO_z,_,_,Leg1_z,Leg2_z,Prop1_z,Prop2_z = self.get_pose(x_pos,z_pos,theta)[:,1]

        if any(x >= self.h_ceil for x in [MO_z,Prop1_z,Prop2_z]):
            impact_flag = True
            Body_contact = True
            
        if Leg1_z >= self.h_ceil:
            impact_flag = True
            Leg1_contact = True

        if Leg2_z >= self.h_ceil:
            impact_flag = True
            Leg2_contact = True

        return impact_flag,[Body_contact,Leg1_contact,Leg2_contact]

    def get_pose(self,x_pos,z_pos,theta):           
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

    def impact_conversion(self,Impact_state,Impact_events):
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

    def beta_prop(self,impact_leg=1):
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

    def beta_landing(self,impact_leg=1):
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
    env = CF_Env_2D()

    env.RENDER = True
    for _ in range(25):
        env.reset()
        done = False
        while not done:
            env.render()
            action = env.action_space.sample()
            # action = np.zeros_like(action)
            obs,reward,done,truncated,info = env.step(action)

    env.close()



                