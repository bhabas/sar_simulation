import gym
from gym import logger,spaces
import stable_baselines3
import math

import numpy as np


class Brake_Val_Env():
    metadata = {'render.modes': ['human']}
    def __init__(self):
        super(Brake_Val_Env, self).__init__()
        self.env_name = "Brake_Val_Cont"

        ## PHYSICS PARAMETERS
        self.dt = 0.02  # seconds between state updates
        self.masscart = 1.0
        self.t_step = 0

        ## POLICY PARAMETERS
        self.x_d = 1.0
        self.Once_flag = False
        self.state = None
        self.C_drag = 0.0

        ## ENV LIMITS
        self.x_threshold = 2.4
        self.t_threshold = 400

        high = np.array(
            [
                self.x_threshold * 2,
                np.finfo(np.float32).max,
            ],
            dtype=np.float32,
        )

        self.action_space = spaces.Box(
            low=-1, high=1, shape=(1,), dtype=np.float32
        )
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        ## RENDERING PARAMETERS
        self.screen_width = 600
        self.screen_height = 400
        self.screen = None
        self.clock = None
        self.isopen = True


    def step(self, action):
        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.action_space.contains(action), err_msg
        assert self.state is not None, "Call reset before using step method."
        self.t_step += 1
        x,x_dot = self.state
        tau,d_ceil = self.obs



        if tau <= 1.0 and self.Once_flag == False:
            self.Once_flag = True
            self.C_drag = action[0]+2
            

        if self.Once_flag != True:
            self.C_drag = 0
        else:
            pass


            

        
        x_acc = (-self.C_drag*x_dot)/self.masscart
        
        ## UPDATE STATE
        x = x + self.dt*x_dot
        x_dot = x_dot + self.dt*x_acc
        self.state = (x, x_dot)

        ## UPDATE OBSERVATION
        d_ceil = self.x_d - x
        tau = d_ceil/x_dot
        self.obs = (tau,d_ceil)

        ## CHECK FOR DONE
        done = bool(
            x > self.x_threshold
            or self.t_step >= self.t_threshold
            or x_dot <= 0.01
        )

        ## CALCULATE REWARD
        if not done:
            reward = 0
        else:
            reward = np.clip(1/np.abs(self.x_d-x+1e-3),0,40)


        return np.array(self.obs,dtype=np.float32), reward, done, {}

    def reset(self):
        ## RESET PHYSICS PARAMS
        self.t_step = 0
        self.Once_flag = False
        self.C_drag = 0.0
        
        ## RESET STATE
        tau_0 = 1.5
        vel_0 = 1.5
        d_ceil_0 = tau_0*vel_0
        pos_0 = self.x_d - d_ceil_0
        self.state = (pos_0,vel_0)

        ## RESET OBSERVATION
        self.obs = (tau_0,d_ceil_0)

        return np.array(self.obs,dtype=np.float32)

    def render(self):
        import pygame
        from pygame import gfxdraw

        ## DEFINE COLORS
        white = (255,255,255)
        black = (0,0,0)
        blue = (29,123,243)
        red = (255,0,0)

        ## INITIATE SCREEN AND CLOCK ON FIRST LOADING
        if self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_width,self.screen_height))
        
        if self.clock is None:
            self.clock = pygame.time.Clock()

        if self.state is None:
            return None
        x = self.state

        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(white)

        ## CREATE TIMESTEP LABEL
        my_font = pygame.font.SysFont(None, 30)
        

        world_width = self.x_threshold * 2
        scale = self.screen_width / world_width

        

        ## CREATE CART
        cartwidth = 50.0
        cartheight = 30.0

        l, r, t, b = -cartwidth / 2, cartwidth / 2, cartheight / 2, -cartheight / 2
        cartx = x[0] * scale + self.screen_width / 2.0  # MIDDLE OF CART
        carty = 100  # TOP OF CART

        gfxdraw.hline(self.surf, 0, self.screen_width, carty,black)
        gfxdraw.vline(self.surf, int(self.x_d*scale + self.screen_width / 2.0), 0, self.screen_height,black)

        ## CREATE CART COORDS AND TRANSLATE TO PROPER LOCATION
        cart_coords = [(l, b), (l, t), (r, t), (r, b)]
        cart_coords = [(c[0] + cartx, c[1] + carty) for c in cart_coords]
        gfxdraw.filled_polygon(self.surf, cart_coords, black)

        if self.Once_flag == True:
            gfxdraw.aacircle(self.surf,int(cartx),int(carty),5,red)
            gfxdraw.filled_circle(self.surf,int(cartx),int(carty),5,red)
        else:
            gfxdraw.aacircle(self.surf,int(cartx),int(carty),5,blue)
            gfxdraw.filled_circle(self.surf,int(cartx),int(carty),5,blue)



        text_t_step = my_font.render(f'Time Step: {self.t_step:03d}', True, black)
        text_action = my_font.render(f'C_drag: {self.C_drag:.03f}', True, black)

        

        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pygame.transform.flip(self.surf, False, True)
        self.screen.blit(self.surf, (0, 0))
        self.screen.blit(text_t_step, (5,5))
        self.screen.blit(text_action, (5,30))




        self.clock.tick(45)
        pygame.display.flip()

        
    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False


if __name__ == '__main__':
    env = Brake_Val_Env()
    for _ in range(5):
        env.reset()
        done = False
        while not done:
            env.render()
            obs,reward,done,info = env.step(env.action_space.sample())


    env.close()
