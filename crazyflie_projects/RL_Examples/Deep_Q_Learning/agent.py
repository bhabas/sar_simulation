from nbformat import read
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

import numpy as np
import matplotlib.pyplot as plt

import gym
from random import sample

from dataclasses import dataclass
from typing import Any


@dataclass
class Sarsd:
    state: Any
    action: int
    reward: float
    next_state: Any
    done: bool


class DQN_Agent:
    def __init__(self,model):
        self.model = model

    def get_actions(self,obs):
        # observations shape is (N,4)

        # q_vals shape (N,2)
        q_vals = self.model(obs)

        return q_vals.max(-1)


class ReplayBuffer:
    def __init__(self, buffer_size=100_000):
        self.buffer_size = buffer_size
        self.buffer = []

    def insert(self,sars):
        self.buffer.append(sars)
        self.buffer = self.buffer[-self.buffer_size:]

    def sample(self,num_samples):
        assert num_samples <= len(self.buffer)
        return sample(self.buffer,num_samples)




class Model(nn.Module):
    def __init__(self, obs_shape, num_actions):
        super(Model,self).__init__()
        self.obs_shape = obs_shape
        self.num_actions = num_actions

        self.net = torch.nn.Sequential(
            torch.nn.Linear(obs_shape[0],256),
            torch.nn.ReLU(),
            torch.nn.Linear(256, num_actions)
        )
        self.opt = optim.Adam(self.net.parameters(),lr = 0.0001)

    def forward(self,x):

        return self.net(x)

    
def update_target_model(model,target):
    target.load_state_dict(model.state_dict())


def train_step(model,state_transitions,target_model,num_actions):
        cur_states = torch.stack(torch.Tensor([s.state for s in state_transitions]))
        rewards = torch.stack(torch.Tensor([s.reward for s in state_transitions]))
        mask = torch.stack(torch.Tensor([0 if s.done else 1 for s in state_transitions]))
        next_states = torch.stack(torch.Tensor([s.next_state for s in state_transitions]))
        actions = [s.action for s in state_transitions]

        with torch.no_grad():
            qvals_next = target_model(next_states).max(-1) # (N,num_actions)
        

        model.optimizer.zero_grad()
        q_vals = model(cur_states) # (N,num_actions)
        one_hot_actions = F.one_hot(torch.LongTensor(actions,num_actions))
        
        loss = (rewards + mask*qvals_next - torch.sum(q_vals*one_hot_actions,-1)).mean()
        loss.backwards()
        model.optimizer.step()


if __name__ == '__main__':
    
    
    env = gym.make("CartPole-v1")
    last_observation = env.reset()


    model = Model(env.observation_space.shape,env.action_space.n)
    target = Model(env.observation_space.shape,env.action_space.n)

    # q_vals = model(torch.FloatTensor(observation))

    rb = ReplayBuffer()

    try:
        while True:
        
        
            action = env.action_space.sample()
            observation,reward,done,info = env.step(action)

            rb.insert(Sarsd(last_observation,action,reward,observation,done))

            last_observation = observation

            if done:
                observation = env.reset()

            if len(rb.buffer) > 5000:
                train_step(model,rb.sample(1000),target,env.action_space.n)

    except KeyboardInterrupt:
        pass

    env.close()