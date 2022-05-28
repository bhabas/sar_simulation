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

    def train(self,state,reward,next_state,done):

        ## ZERO NETWORK GRADIENTS
        self.actor.optimizer.zero_grad()
        self.critic.optimizer.zero_grad()

        ## CALCULATE STATE VALUES AND REWARDS
        V_state = self.critic.forward(state)
        V_next_state = self.critic.forward(next_state)
        reward = T.tensor(reward,dtype=T.float).to(self.actor.device)

        ## TD ADVANTAGE ESTIMATE
        delta = reward + self.gamma*V_next_state*(1-int(done)) - V_state # If episode is over then discard future term

        ## CALCULATE LOSS
        actor_loss = -self.log_probs*delta # Minimize policy gradient
        critic_loss = delta**2  # Minimize loss

        ## UPDATE NETWORKS
        (actor_loss + critic_loss).backward()
        self.actor.optimizer.step()
        self.critic.optimizer.step()


if __name__ == '__main__':

    env = gym.make('MountainCarContinuous-v0')
    agent = Agent(alpha=0.0001,beta=0.0001,input_dims=[2],gamma=0.99,
            layer1_size=256,layer2_size=256)

    score_history = []

    num_episodes = 50
    RENDER = False

    for i in range(1,num_episodes):
        done = False
        score = 0
        state = env.reset()

        while not done:
            if RENDER:
                env.render()

            action = np.array(agent.choose_action(state)).reshape((1,))
            next_state,reward,done,info = env.step(action)
            agent.train(state,reward,next_state,done)
            state = next_state
            score += reward

        score_history.append(score)
        print(f"EPISODE: {i} Score: {score:.2f} MA_Reward: {np.nanmean(score_history[-5:]):.2f}")
        if i%10 == 0:
            RENDER = True
        else:
            RENDER = False

    # PLOT RESULTS
    plt.plot(range(num_episodes),score_history)
    plt.show()
