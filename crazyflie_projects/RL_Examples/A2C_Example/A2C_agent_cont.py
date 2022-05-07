import gym
from gym import wrappers
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import torch as T
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

class GenericNetwork(nn.Module):
    def __init__(self,lr,input_dims,fc1_dims,fc2_dims,n_actions):
        super(GenericNetwork,self).__init__()
        self.lr = lr
        self.input_dims = input_dims
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.n_actions = n_actions


        self.fc1 = nn.Linear(*self.input_dims,self.fc1_dims)
        self.fc2 = nn.Linear(self.fc1_dims,self.fc2_dims)
        self.fc3 = nn.Linear(self.fc2_dims,self.n_actions)

        self.optimizer = optim.Adam(self.parameters(),lr=self.lr)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device) # Send network to device

    def forward(self, observation):
        state = T.tensor(observation,dtype=T.float).to(self.device)
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)

        return x

class Agent(object):
    def __init__(self,alpha,beta,input_dims,gamma=0.99,n_actions=2,
                    layer1_size=64,layer2_size=64,n_outputs=1):
        self.gamma = gamma
        self.log_probs = None
        self.n_outputs = n_outputs
        self.actor = GenericNetwork(alpha,input_dims,layer1_size,layer2_size,n_actions=2)   # Outputs mu and var
        self.critic = GenericNetwork(beta,input_dims,layer1_size,layer2_size,n_actions=1)   # Outputs V_state

    def choose_action(self,observation):
        mu,sigma = self.actor.forward(observation)
        sigma = T.exp(sigma) # Ensures sigma is positive (TEST: softplus)

        ## SAMPLE ACTION FROM DISTRIBUTION
        action_probs = T.distributions.Normal(mu,sigma)
        probs = action_probs.sample(sample_shape=T.Size([self.n_outputs]))
        action = T.tanh(probs) # Clamp to [-1,1] range

        ## CONVERT PROB. OF ACTION TO LOG PROB.
        self.log_probs = action_probs.log_prob(probs).to(self.actor.device)

        return action.item()

    def learn(self,state,reward,new_state,done):

        ## ZERO NETWORK GRADIENTS
        self.actor.optimizer.zero_grad()
        self.critic.optimizer.zero_grad()

        ## CALCULATE STATE VALUES AND REWARDS
        critic_value = self.critic.forward(state)
        critic_value_ = self.critic.forward(new_state)
        reward = T.tensor(reward,dtype=T.float).to(self.actor.device)

        ## CALC ADVANTAGE
        delta = reward + self.gamma*critic_value*(1-int(done)) - critic_value_ # If episode is over then discard future term

        ## CALCULATE LOSS
        actor_loss = -self.log_probs*delta # Minimize policy gradient
        critic_loss = delta**2  # Minimize loss

        ## UPDATE NETWORKS
        (actor_loss + critic_loss).backward()
        self.actor.optimizer.step()
        self.critic.optimizer.step()


if __name__ == '__main__':
    agent = Agent(alpha=5e-6,beta=10e-5,input_dims=[2],gamma=0.99,
            layer1_size=256,layer2_size=256)

    env = gym.make('MountainCarContinuous-v0')
    score_history = []
    num_episodes = 100
    for i in range(num_episodes):
        done = False
        score = 0
        observation = env.reset()

        while not done:
            action = np.array(agent.choose_action(observation)).reshape((1,))
            observation_,reward,done,info = env.step(action)
            agent.learn(observation,reward,observation_,done)
            observation = observation_
            score += reward

        score_history.append(score)
        if i%1 == 0:
            RENDER = True
            print(f"EPISODE: {i} Score: {score}")
        else:
            RENDER = False
