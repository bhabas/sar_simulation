import numpy as np

import torch as T
import torch.nn as nn
import torch.optim as optim
from torch.distributions.categorical import Categorical

import os

from CF_Env import CF_Env

DEVICE="cuda:0"
EPISODES = 15_000
GAMMA=0.999
LR = 0.001
RENDER=True


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
                nn.Linear(fc2_dims, 2),
                nn.Softmax(dim=-1)
        )

        self.optimizer = optim.Adam(self.parameters(), lr=alpha)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state):

        dist = self.actor(state)
        dist = Categorical(dist)
        
        return dist

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

        ## SAMPLE ACTION FROM POLICY NETWORK
        dist = self.actor(state)
        action = dist.sample()

        log_prob = T.squeeze(dist.log_prob(action)).item()
        action = T.squeeze(action).item()

        return action,log_prob,None

    def train(self,rewards,log_probs):

        ## CALCULATE DISCOUNTED RETURN FOR EACH TIME-STEP
        Discounted_Returns = []
        for ii in range(len(rewards)):
            G = 0.0
            for k,r in enumerate(rewards[ii:]):
                G += (GAMMA**k)*r
            Discounted_Returns.append(G)

        Discounted_Returns = T.tensor(Discounted_Returns, dtype=T.float,requires_grad=True).to(DEVICE)
        log_prob = T.tensor(log_probs,dtype=T.float,requires_grad=True).to(DEVICE)
        policy_gradient = -log_prob*(Discounted_Returns)

        self.actor.optimizer.zero_grad()
        policy_gradient.sum().backward()
        self.actor.optimizer.step()


if __name__ == '__main__':

 
    agent = Agent()
    env = CF_Env()

    agent.My = -7e-3
    agent.tau_c = 0.13

    ## POLICY CONDITIONS
    alpha = 0.0003
    EPISODES = 200
    RENDER = True
    ep_score = []


    episode = 0
    ## RESET ENVIRONMENT
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
    print(f"ep: {episode}")
    states,actions,log_probs,vals,rewards,dones = env.solveODE(agent,IC=IC,t_span=[0,1.5])
    ep_score.append(np.sum(rewards))

    if episode%10 == 0:
        env.animateTraj(states)
        print(f"EPISODE: {episode} Score: {np.sum(rewards):2f} MA_Reward: {np.nanmean(ep_score[-20:]):.2f}")
        
            


        ## TRAIN AGENT
        # agent.remember(observation, action, prob, val, reward, done)
        # agent.train(rewards,log_probs)


