import numpy as np

import torch as T
import torch.nn as nn
import torch.optim as optim

import os

from CF_Env import CF_Env


class ActorNetwork(nn.Module):
    def __init__(self, input_dims, alpha, fc1_dims=256, fc2_dims=256,
            chkpt_dir='/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/CF_Env'):
        super(ActorNetwork, self).__init__()

        self.checkpoint_file = os.path.join(chkpt_dir, 'actor_torch_ppo')
        self.actor = nn.Sequential(
                nn.Linear(input_dims, fc1_dims),
                nn.ReLU(),
                nn.Linear(fc1_dims, fc2_dims),
                nn.ReLU(),
                nn.Linear(fc2_dims, 1)
        )

        self.optimizer = optim.Adam(self.parameters(), lr=alpha)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state):

        return self.actor(state)

    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))


class Agent:
    def __init__(self,input_dims=3,alpha=0.001):
        self.My = -6.7e-3
        self.tau_c = 0.24
        
        self.actor = ActorNetwork(input_dims,alpha)


    def save_models(self):
        print('... saving models ...')
        self.actor.save_checkpoint()

    def load_models(self):
        print('... loading models ...')
        self.actor.load_checkpoint()

    def choose_action(self, observation):
        state = T.tensor([observation], dtype=T.float).to(self.actor.device)

        # ## SAMPLE ACTION FROM POLICY NETWORK
        # dist = self.actor(state)
        # action = dist.sample()

        # log_prob = T.squeeze(dist.log_prob(action)).item()
        # action = T.squeeze(action).item()

        ## CALC STATE VALUE FROM CRITIC NETWORK
        value = self.critic(state)
        value = T.squeeze(value).item()

        # return action, log_prob, value

        action = self.actor(state)
        action = T.squeeze(action).item()

        # print(action)

        return self.tau_c,self.My

    def remember(self):
        pass
       

if __name__ == '__main__':

 
    agent = Agent()
    env = CF_Env()

    agent.My = -7e-3
    agent.tau_c = 0.14

    ## POLICY CONDITIONS
    alpha = 0.0003
    EPISODES = 3
    RENDER = True


    for i in range(EPISODES):

        ## RESET ENVIRONMENT
        score = 0

        ## VELOCITY CONDITIONS
        vel = 3.0
        phi = 60

        phi_rad = np.radians(phi)
        vz = vel*np.sin(phi_rad)
        vx = vel*np.cos(phi_rad)

        ## INITIALIZE STARTING CONDITIONS
        tau_0 = 0.6
        d_0 = vz*tau_0
        z_0 = (env.h_ceiling - d_0)

        IC = [0, vx, z_0, vz, 0, 0]

        
        ## EXECUTE ENVIRONMENT/AGENT
        states,actions,probs,vals,rewards,dones = env.solveODE(agent,IC=IC,t_span=[0,1.5])

        if RENDER:
            env.animateTraj(states)
            


        ## TRAIN AGENT
        # agent.remember(observation, action, prob, val, reward, done)
        # agent.learn()


