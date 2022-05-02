import gym
import pandas as pd
import numpy as np


import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical

DEVICE="cuda:0"
EPISODES = 15_000
GAMMA=0.9
LR = 0.001
RENDER=True


class ReinforceModel(nn.Module):
    def __init__(self,input_size=4,output_size=2):
        super(ReinforceModel,self).__init__()

        self.fc1 = nn.Linear(input_size,64)
        self.fc2 = nn.Linear(64,output_size)

        self.optimizer = optim.Adam(self.parameters(), lr=LR)
        self.to(DEVICE)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.softmax(self.fc2(x),dim=1)

        return x
            

class Agent():
    def __init__(self):

        self.Policy_NN = ReinforceModel(input_size=4,output_size=2)

    def choose_action(self,state):

        ## FIND ACTION PROBABILITIES FOR CURRENT STATE 
        state = torch.tensor(state.reshape(1,-1),dtype=torch.float).to(DEVICE) # Convert state to tensor
        action_probs = self.Policy_NN.forward(state) # Probability for each action

        ## SAMPLE FROM PROBABILITY DISTRIBUTION
        dist = Categorical(action_probs)
        action = dist.sample()

        return action.item(),dist.log_prob(action).squeeze(0)

    def train(self,rewards,log_probs):

        ## CALCULATE DISCOUNTED RETURN FOR EACH TIME-STEP
        Discounted_Returns = []
        for ii in range(len(rewards)):
            G = 0.0
            for k,r in enumerate(rewards[ii:]):
                G += (GAMMA**k)*r
            Discounted_Returns.append(G)

        Discounted_Returns = torch.tensor(Discounted_Returns, dtype=torch.float).to(DEVICE)
        log_prob = torch.stack(log_probs)
        policy_gradient = -log_prob*Discounted_Returns

        self.Policy_NN.optimizer.zero_grad()
        policy_gradient.sum().backward()
        self.Policy_NN.optimizer.step()



if __name__ == '__main__':

    env = gym.make("CartPole-v1")
    agent = Agent()
    ep_score = []

    for episode in range(EPISODES):

        done = False
        state = env.reset()

        log_probs = []
        actions = []
        rewards = []
        dones = []
        states = []

        while not done:
            
            if RENDER:
                env.render()
            
            ## STEP THROUGH ENVIRONMENT
            action,log_prob = agent.choose_action(state)
            next_state,reward,done,_ = env.step(action)

            ## RECORD REWARDS AND LOG-PROBABILITY OF CHOSEN ACTION
            log_probs.append(log_prob)
            rewards.append(reward)

            if done:
                ep_score.append(np.sum(rewards))
                if episode%50 == 0:
                    RENDER = True
                    print(f"EPISODE: {episode} Score: {np.sum(rewards)} MA_Reward: {np.nanmean(ep_score[-30:]):.2f}")
                else:
                    RENDER = False

            state = next_state

        agent.train(rewards,log_probs)

        
        
