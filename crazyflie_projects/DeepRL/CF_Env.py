import numpy as np
import pygame
from pygame import gfxdraw
import os

from gym import spaces



class CF_Env():
    metadata = {'render.modes': ['human']}
    def __init__(self):
        super(CF_Env, self).__init__()
        self.env_name = "CF_Env"

        ## PHYSICS PARAMETERS
        self.dt = 0.005  # seconds between state updates
        self.t_step = 0
    

        ## SET DIMENSIONAL CONSTRAINTS 
        self.h_ceil = 2.1
        g = 9.81        # Gravity [m/s^2]
        PD = 0.115/2    # Prop Distance from COM [m]
        I_G = 16.5717e-6    # Moment of Intertia [kg*m^2]
        L = 0.1
        gamma = np.deg2rad(30)
        M_G = 0.035 
        M_L = 0.002
        e = 0.018
        self.params = (L,e,gamma,M_G,M_L,g,PD,I_G)




        self.state = None
        self.obs = None

        self.RENDER = False

        self.Flip_flag = False
        self.Impact_flag = False
        self.Moment_flag = False
        self.Impact_events = []
        self.Stages = [0,0,0,0,0,0]


        ## ENV LIMITS
        self.world_width = 4.0  # [m]
        self.world_height = 3.0 # [m]
        self.t_threshold = 400

        high = np.finfo(np.float32).max
        self.observation_space = spaces.Box(-high, high, shape=(2,),dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-1,0]), high=np.array([1,4]), shape=(2,), dtype=np.float32)

        ## RENDERING PARAMETERS
        self.screen_width = 1000 # [pixels]
        self.screen_height = self.screen_width*self.world_height/self.world_width
        self.screen = None
        self.clock = None
        self.isopen = True

    def step(self,action):

        # err_msg = f"{action!r} ({type(action)}) invalid"
        # assert self.action_space.contains(action), err_msg
        # assert self.state is not None, "Call reset before using step method."
        
        self.t_step += 1
        x,x_dot,z,z_dot,theta,dtheta = self.state
        Tau,d_ceil = self.obs
        (L,e,gamma,M_G,M_L,g,PD,I_G) = self.params

        done = False
        reward = np.nan
    
        if self.Flip_flag == False:

            ##############################
            #     CONSTANT VEL TRAJ.
            ##############################

            ## UPDATE STATE
            x_acc = 0
            x = x + self.dt*x_dot
            x_dot = x_dot + self.dt*x_acc

            z_acc = 0
            z = z + self.dt*z_dot
            z_dot = z_dot + self.dt*z_acc
            
            theta_acc = 0
            theta = theta + self.dt*dtheta
            dtheta = dtheta + self.dt*theta_acc

            self.state = (x,x_dot,z,z_dot,theta,dtheta)

            ## UDPATE OBSERVATION
            d_ceil = self.h_ceil - z
            Tau = d_ceil/z_dot

            self.obs = (Tau,d_ceil)

        
        if Tau <= 0.14 or self.Flip_flag == True:
            self.Flip_flag = True
            self.Tau_thr = Tau
            My = -4e-3

            self.reward,done = self.finish_sim(My)

            

        

        return np.array(self.obs,dtype=np.float32),reward,done,{}

    def finish_sim(self,My):
        
        (L,e,gamma,M_G,M_L,g,PD,I_G) = self.params
        done = False

        while not done:

            if self.Impact_flag == False:

                self.t_step += 1
                x,x_dot,z,z_dot,theta,dtheta = self.state

                ##############################
                #     EXECUTE BODY MOMENT
                ##############################

                ## CHECK IF PAST 90 DEG
                if np.abs(theta) < np.deg2rad(90):
                    My = My
                else:
                    My = 0
                
                x_acc = -My/(M_G*PD)*np.cos(np.pi/2-theta)
                x = x + self.dt*x_dot
                x_dot = x_dot + self.dt*x_acc

                z_acc = -My/(M_G*PD)*np.sin(np.pi/2-theta) - g
                z = z + self.dt*z_dot
                z_dot = z_dot + self.dt*z_acc
                
                theta_acc = My/I_G
                theta = theta + self.dt*dtheta
                dtheta = dtheta + self.dt*theta_acc

                self.Impact_flag,self.Impact_events = self.impact_conditions(x,z,theta)
                self.state = (x,x_dot,z,z_dot,theta,dtheta)


                if self.RENDER:
                    self.render()

            elif self.Impact_flag == True:

                self.t_step += 1
                x,x_dot,z,z_dot,theta,dtheta = self.state

                ## BODY CONTACT
                if self.Impact_events[0] == True:
                    body_contact = True
                    pad_contacts = 0
                    done = True

                ## LEG 1 CONTACT
                elif self.Impact_events[1] == True:
                    
                    beta_0,dbeta_0 = self.impact_Conversion(self.state,self.Impact_events)
                    beta = beta_0
                    dbeta = dbeta_0

                    l = L/2 # Half leg length [m]
                    I_C = M_G*(4*l**2 + e**2 + 4*l*e*np.sin(gamma)) + I_G   # Inertia about contact point

                    ## FIND IMPACT COORDS
                    r_G_C1_0 = np.array(
                                [[-(L+e*np.sin(gamma))*np.cos(beta_0)+e*np.cos(gamma)*np.sin(beta_0)],
                                [-(L+e*np.sin(gamma))*np.sin(beta_0)-e*np.cos(gamma)*np.cos(beta_0)]])

                    r_O_G = np.array([[x],[z]])
                    r_O_C1 = r_O_G - r_G_C1_0

                    ## SOLVE SWING ODE
                    while not done:

                        beta_acc = M_G*g/I_C*(2*l*np.cos(beta) - e*np.sin(beta-gamma))
                        beta = beta + self.dt*dbeta
                        dbeta = dbeta + self.dt*beta_acc

                        if beta > self.beta_landing(impact_leg=1):
                            body_contact = False
                            pad_contacts = 4
                            done = True

                        elif beta < self.beta_prop(impact_leg=1):
                            body_contact = True
                            pad_contacts = 2
                            done = True

                        ## SOLVE FOR SWING BEHAVIOR IN GLOBAL COORDINATES
                        r_G_C1 = np.array(
                                    [[-(L+e*np.sin(gamma))*np.cos(beta)+e*np.cos(gamma)*np.sin(beta)],
                                    [-(L+e*np.sin(gamma))*np.sin(beta)-e*np.cos(gamma)*np.cos(beta)]])

                        theta = -((np.pi/2-gamma) + beta) # Convert beta in (e^) to theta in (G^)
                        x = r_O_C1[0,0] + r_G_C1[0,0]  # x
                        z = r_O_C1[1,0] + r_G_C1[1,0]  # z


                        self.state = (x,_,z,_,theta,_)
                        if self.RENDER:
                            self.render()

                ## LEG 2 CONTACT
                elif self.Impact_events[2] == True:

                    beta_0,dbeta_0 = self.impact_Conversion(self.state,self.Impact_events)
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

                        beta_acc = M_G*g/I_C*(2*l*np.cos(beta) + e*np.sin(beta+gamma))
                        beta = beta + self.dt*dbeta
                        dbeta = dbeta + self.dt*beta_acc

                        if beta < self.beta_landing(impact_leg=2):
                            body_contact = False
                            pad_contacts = 4
                            done = True

                        elif beta > self.beta_prop(impact_leg=2):
                            body_contact = True
                            pad_contacts = 2
                            done = True

                        ## SOLVE FOR SWING BEHAVIOR IN GLOBAL COORDINATES
                        r_G_C2 = np.array(
                        [[-(L+e*np.sin(gamma))*np.cos(beta)-e*np.cos(gamma)*np.sin(beta)],
                        [-(L+e*np.sin(gamma))*np.sin(beta)+e*np.cos(gamma)*np.cos(beta)]])

                        theta = -((np.pi/2+gamma) + beta) # Convert beta in (e^) to theta in (G^)
                        x = r_O_C2[0,0] + r_G_C2[0,0]  # x
                        z = r_O_C2[1,0] + r_G_C2[1,0]  # z


                        self.state = (x,_,z,_,theta,_)
                        if self.RENDER:
                            self.render()





            


        return 0.0,done
            
            


    def reset(self):
        self.Flip_flag = False
        self.Impact_flag = False
        self.Impact_events = []



        x, x_dot = 0.0, 1.5
        z, z_dot = 0.7, 2.5
        theta,dtheta = 0.0, 0.0
        self.state = (x,x_dot,z,z_dot,theta,dtheta)

        d_ceil = (self.h_ceil - z)
        tau = d_ceil/z_dot
        self.obs = (tau,d_ceil)

    def render(self,mode=None):

        x = 500
        y = 700
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x,y)
        

        def c2p(Pos): ## CONVERTS COORDINATES TO PIXEL LOCATION      

            x_offset = 1
            y_offset = 0.3

            scale_x = self.screen_width/self.world_width
            scale_y = self.screen_height/self.world_height
            
            x_p = (x_offset+Pos[0])*scale_x
            y_p = (y_offset+Pos[1])*scale_y
            
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

        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE)

        ## CREATE TIMESTEP LABEL
        my_font = pygame.font.SysFont(None, 30)
        text_t_step = my_font.render(f'Time Step: {self.t_step:03d}', True, BLACK)

        x,x_dot,z,z_dot,theta,dtheta = self.state

        Pose = self.get_pose(x,z,theta)

        ## CREATE QUADROTOR
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[1]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[2]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[3]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[4]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[5]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[6]),width=3)

        pygame.draw.circle(self.surf,RED,c2p(Pose[0]),5)

        if self.Flip_flag == True:
            pygame.draw.circle(self.surf,RED,c2p(Pose[0]),5)
        else:
            pygame.draw.circle(self.surf,BLUE,c2p(Pose[0]),5)




        ## DRAW FLOOR LINE
        pygame.draw.line(self.surf,BLACK,c2p((-5,0)),c2p((5,0)),width=2)

        ## DRAW CEILING LINE
        pygame.draw.line(self.surf,BLACK,c2p((-5,self.h_ceil)),c2p((5,self.h_ceil)),width=2)

        

        
        

        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pygame.transform.flip(self.surf, False, True)
        self.screen.blit(self.surf,     (0,0))
        self.screen.blit(text_t_step,   (5,5))




        self.clock.tick(60)
        pygame.display.flip()

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

    def impact_Conversion(self,Impact_state,Impact_events):
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
            H_vy = M_G*vz*(2*l*np.sin(gamma-theta)+e*np.cos(theta))     # Angular momentum from vel_y


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

    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False


if __name__ == '__main__':
    env = CF_Env()
    env.RENDER = True
    for _ in range(25):
        done = False
        env.reset()
        while not done:
            env.render()
            obs,reward,done,info = env.step(np.array([1.0,2.0]))


    env.close()
