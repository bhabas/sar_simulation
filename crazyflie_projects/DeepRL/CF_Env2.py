import gym
from gym import logger,spaces
import stable_baselines3
import math
import pygame
from pygame import gfxdraw
import os
import numpy as np


class CF_Env3():
    metadata = {'render.modes': ['human']}
    def __init__(self):
        super(CF_Env3, self).__init__()
        self.env_name = "CF_Env3"

        ## PHYSICS PARAMETERS
        self.dt = 0.01  # seconds between state updates
        self.t_step = 0
        self.RENDER = False

        ## POLICY PARAMETERS
        self.Once_flag = False
        self.state = None
        self.Tau_thr = 0.0
        self.reward = 0.0

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

        high = np.array(
            [
                2.4*2,
                np.finfo(np.float32).max,
            ],
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-1]), high=np.array([1]), shape=(1,), dtype=np.float32)

        ## ENV LIMITS
        self.world_width = 4.0  # [m]
        self.world_height = 3.0 # [m]
        self.t_threshold = 400


        ## RENDERING PARAMETERS
        self.screen_width = 1000
        self.screen_height = self.screen_width*self.world_height/self.world_width
        self.screen = None
        self.clock = None
        self.isopen = True


    def step(self, action):
        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.action_space.contains(action), err_msg
        assert self.state is not None, "Call reset before using step method."
        
        z,vz,x,vx,theta,dtheta = self.state
        Tau,d_ceil = self.obs

        # if Tau <= 0.14:
        #     action[0] = -1
        # else:
        #     action[0] = 1

        ## ONCE ACTIVATED SAMPLE ACTION       
        if action[0] >= 0:

            ## UPDATE STATE
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

            self.state = (z,vz,x,vx,theta,dtheta)

            ## UPDATE OBSERVATION
            d_ceil = self.h_ceil - z
            Tau = d_ceil/vz
            self.obs = (Tau,d_ceil)

            if d_ceil <= self.d_min:
                self.d_min = d_ceil

            ## CHECK FOR DONE
            done = bool(
                self.t_step >= self.t_threshold
                or z > 2.4
                or z < 0.2
            )

            ## CALCULATE REWARD
            if not done:
                reward = 0
            else:
                reward = np.clip(1/np.abs(d_ceil+1e-3),0,10)

        elif action[0] < 0:
            self.Once_flag = True
            self.Tau_thr = Tau
            done = True
            reward = self.finish_sim()



        return np.array(self.obs,dtype=np.float32), reward, done, {}

    def finish_sim(self):

        
        done = False

        while not done:

            ## UPDATE STATE
            self.t_step += 1
            z,vz,x,vx,theta,dtheta = self.state

            z_acc = -9.81
            z = z + self.dt*vz
            vz = vz + self.dt*z_acc

            x_acc = 0.0
            x = x + self.dt*vx
            vx = vx + self.dt*x_acc

            theta_acc = 0.0
            theta = theta + self.dt*dtheta
            dtheta = dtheta + self.dt*theta_acc

            self.state = (z,vz,x,vx,theta,dtheta)

            ## UPDATE OBSERVATION
            d_ceil = self.h_ceil - z
            Tau = d_ceil/vz

            if d_ceil <= self.d_min:
                self.d_min = d_ceil

            ## CHECK FOR DONE
            done = bool(
                self.t_step >= self.t_threshold
                or z > 2.4
                or z < 0.2
            )

            if self.RENDER:
                self.render()
        
        self.reward = np.clip(1/np.abs(self.d_min+1e-3),0,50)
        return self.reward

    def reset(self):
        ## RESET PHYSICS PARAMS
        self.t_step = 0
        self.Once_flag = False
        self.Tau_thr = 0.0
        self.d_min = 500
        
        ## RESET STATE
        z_0 = 0.4
        vz_0 = np.random.uniform(low=0.5,high=3.0)
        # vz_0 = 2.0

        x = 0.0
        vx = 1.0

        theta = 0.0
        dtheta = -2.0
        self.state = (z_0,vz_0,x,vx,theta,dtheta)




        ## RESET OBSERVATION
        d_ceil_0 = self.h_ceil - z_0
        Tau_0 = d_ceil_0/vz_0
        self.obs = (Tau_0,d_ceil_0)

        self.start_vals = (vz_0,Tau_0)

        return np.array(self.obs,dtype=np.float32)

    def render(self,mode=None):

        z = 500
        y = 700
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (z,y)
        
        def c2p(Pos): ## CONVERTS COORDINATES TO PIXEL LOCATION      

            x_offset = 2
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
        z,vz,x,vx,theta,dtheta = self.state

        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(WHITE)

        ## CREATE TIMESTEP LABEL
        my_font = pygame.font.SysFont(None, 30)


        Pose = self.get_pose(x,z,theta)

        ## CREATE QUADROTOR
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[1]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[2]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[3]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[4]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[5]),width=3)
        pygame.draw.line(self.surf,BLACK,c2p(Pose[0]),c2p(Pose[6]),width=3)
    
        

        pygame.draw.line(self.surf,BLACK,c2p((0,-5)),c2p((0,5)),width=1)


        pygame.draw.line(self.surf,BLACK,c2p((-5,self.h_ceil)),c2p((5,self.h_ceil)),width=2)
        pygame.draw.line(self.surf,BLACK,c2p((-5,0)),c2p((5,0)),width=2)


        

        pygame.draw.circle(self.surf,BLACK,c2p((x,z)),radius=7,width=3)
        if self.Once_flag == True:
            pygame.draw.circle(self.surf,RED,c2p((x,z)),radius=4,width=0)
        else:
            pygame.draw.circle(self.surf,BLUE,c2p((x,z)),radius=4,width=0)



        text_t_step = my_font.render(f'Time Step: {self.t_step:03d}', True, BLACK)
        text_Vz = my_font.render(f'Vz_0: {self.start_vals[0]:.02f}', True, BLACK)
        text_Tau_0 = my_font.render(f'Tau_0: {self.start_vals[1]:.03f}', True, BLACK)
        text_Tau_trg = my_font.render(f'Tau_trg: {self.Tau_thr:.03f}', True, BLACK)
        text_d_min = my_font.render(f'd_min: {self.d_min:.03f}', True, BLACK)
        text_reward = my_font.render(f'Prev Reward: {self.reward:.01f}',True, BLACK)


        
        

        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pygame.transform.flip(self.surf, False, True)
        self.screen.blit(self.surf,     (0,0))
        self.screen.blit(text_t_step,   (5,5))
        self.screen.blit(text_Vz,       (5,30))
        self.screen.blit(text_Tau_0,    (5,55))
        self.screen.blit(text_Tau_trg,  (5,80))
        self.screen.blit(text_d_min,    (5,105))
        self.screen.blit(text_reward,   (5,130))




        self.clock.tick(60)
        pygame.display.flip()

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
    env = CF_Env3()
    env.RENDER = True
    for _ in range(25):
        env.reset()
        done = False
        while not done:
            env.render()
            obs,reward,done,info = env.step(env.action_space.sample())


    env.close()
