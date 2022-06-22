import numpy as np
import pygame
from pygame import gfxdraw



class CF_Env():
    metadata = {'render.modes': ['human']}
    def __init__(self):
        super(CF_Env, self).__init__()
        self.env_name = "CF_Env"

        ## PHYSICS PARAMETERS
        self.dt = 0.01  # seconds between state updates
        self.t_step = 0
        self.RENDER = False

        self.Flip_flag = False
        self.Impact_flag = False
        self.Moment_flag = False

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



        ## ENV LIMITS
        self.world_width = 4.0  # [m]
        self.world_height = 3.0 # [m]
        self.t_threshold = 400

        # high = np.finfo(np.float32).max
        # self.observation_space = spaces.Box(-high, high, shape=(3,),dtype=np.float32)
        # self.action_space = spaces.Box(low=np.array([-1,0]), high=np.array([1,4]), shape=(2,), dtype=np.float32)

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
    
        if Tau > 0.5 and self.Flip_flag == False:

            ##############################
            #     CONSTANT VEL TRAJ.
            ##############################
            x_acc = 0
            x = x + self.dt*x_dot
            x_dot = x_dot + self.dt*x_acc

            z_acc = 0
            z = z + self.dt*z_dot
            z_dot = z_dot + self.dt*z_acc
            
            theta_acc = 0
            theta = theta + self.dt*dtheta
            dtheta = dtheta + self.dt*theta_acc

            impact_flag,impact_events = self.impact_conditions(x,z,theta)

        if Tau <= 1.0 or self.Flip_flag == True:
            self.Flip_flag = True
            self.Tau_thr = Tau
            My = 1e-3

            if np.deg2rad(90)-np.abs(theta) < 0:
                self.Moment_flag = True
            
            ##############################
            #     EXECUTE BODY MOMENT
            ##############################
            if self.Moment_flag == False:
                x_acc = -My/(M_G*PD)*np.cos(np.pi/2-theta)
                x = x + self.dt*x_dot
                x_dot = x_dot + self.dt*x_acc

                z_acc = -My/(M_G*PD)*np.sin(np.pi/2-theta) - g
                z = z + self.dt*z_dot
                z_dot = z_dot + self.dt*z_acc
                
                theta_acc = My/I_G
                theta = theta + self.dt*dtheta
                dtheta = dtheta + self.dt*theta_acc
            
            else:
                x_acc = 0
                x = x + self.dt*x_dot
                x_dot = x_dot + self.dt*x_acc

                z_acc = -g
                z = z + self.dt*z_dot
                z_dot = z_dot + self.dt*z_acc
                
                theta_acc = 0
                theta = theta + self.dt*dtheta
                dtheta = dtheta + self.dt*theta_acc
            




        d_ceil = self.h_ceil - x
        Tau = d_ceil/z_dot

        self.obs = (Tau,d_ceil)
        self.state = (x,x_dot,z,z_dot,theta,dtheta)

    


    def reset(self):
        self.Flip_flag = False


        x, x_dot = 0.0, 1.0
        z, z_dot = 0.2, 1.0
        theta,dtheta = 0.0, 0.0
        self.state = (x,x_dot,z,z_dot,theta,dtheta)

        d_ceil = (self.h_ceil - z)
        tau = d_ceil/z_dot
        self.obs = (tau,d_ceil)

    def render(self,mode=None):
        

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

        
    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False


if __name__ == '__main__':
    env = CF_Env()
    env.RENDER = True
    for _ in range(5):
        done = False
        env.reset()
        while not done:
            env.render()
            env.step(np.array([1.0,2.0]))


    env.close()
