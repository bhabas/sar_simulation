import gym
from gym import logger,spaces
from stable_baselines3.common.env_checker import check_env
import math

import numpy as np


class Cont_Value_Pred_Env():
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(Cont_Value_Pred_Env, self).__init__()

        self.gravity = 9.8
        self.masscart = 1.0
        self.force_mag = 10.0
        self.tau = 0.02  # seconds between state updates
        self.x_d = 1.0
        self.t_step = 0
        self.action = 0
        self.x = 0

        # Angle at which to fail the episode
        self.x_threshold = 2.4
        self.t_threshold = 500

        # Angle limit set to 2 * theta_threshold_radians so failing observation
        # is still within bounds.
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

        self.screen_width = 600
        self.screen_height = 400
        self.screen = None
        self.clock = None
        self.isopen = True
        self.state = None

        self.steps_beyond_done = None

    def step(self, action):
        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.action_space.contains(action), err_msg
        assert self.state is not None, "Call reset before using step method."
        self.t_step += 1
        x,x_dot = self.state

        
        self.action = action[0]
        force = 0.0
        self.x = x
        

        C = 0.0

        T = 100
        w = 2*np.pi/T
        x = np.sin(w*self.t_step)
        x_dot = np.cos(w*self.t_step)

        self.state = (x, x_dot)


        done = bool(
            x < -self.x_threshold
            or x > self.x_threshold
            or self.t_step >= self.t_threshold
        )

        if not done:
            reward = np.clip(1/np.abs(x-self.action+1e-3),-10,10)

        else:
            reward = 0.0


        return np.array(self.state,dtype=np.float32), reward, done, {}

    def reset(self):
        self.t_step = 0
        self.state = np.random.uniform(low=-0.1,high=0.1,size=(2,))
        self.state = [-1,0]

        return np.array(self.state,dtype=np.float32)

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


        gfxdraw.aacircle(self.surf,int(cartx),int(carty),5,blue)
        gfxdraw.filled_circle(self.surf,int(cartx),int(carty),5,blue)



        text_t_step = my_font.render(f'Time Step: {self.t_step:03d}', True, black)
        text_action = my_font.render(f'Val: {self.x-self.action:.03f}', True, black)

        

        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pygame.transform.flip(self.surf, False, True)
        self.screen.blit(self.surf, (0, 0))
        self.screen.blit(text_t_step, (5,5))
        self.screen.blit(text_action, (5,30))




        self.clock.tick(60)
        pygame.display.flip()

        
    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False


if __name__ == '__main__':

    env = Cont_Value_Pred_Env()
    env.reset()

    done = False
    while not done:
        env.render()
        obs,reward,done,info = env.step(env.action_space.sample())


    env.close()
