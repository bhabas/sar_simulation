import gym
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Categorical

DEVICE = "cuda:0"
EPISODES = 3000
GAMMA = 0.99
NUM_STEPS = 300
LR = 3e-4
RENDER = True


class A2C(nn.Module):
    def __init__(self,num_inputs,num_actions,hidden_size,learning_rate=LR):
        super(A2C,self).__init__()

        self.num_actions = num_actions
        self.num_inputs = num_inputs

        ## DEFINE ACTOR NETWORK
        self.actor_linear1 = nn.Linear(num_inputs,hidden_size)
        self.actor_linear2 = nn.Linear(hidden_size,num_actions) # Function for Pi(a|s)

        ## DEFINE CRITIC NETWORK
        self.critic_linear1 = nn.Linear(num_inputs,hidden_size)
        self.critic_linear2 = nn.Linear(hidden_size,1) # Function for V(s)

        self.optimizer = optim.Adam(self.parameters(),lr=learning_rate)
        self.to(DEVICE)

    def forward(self,state):

        state = torch.tensor(state.reshape(1,-1),dtype=torch.float).to(DEVICE) # Convert state to tensor

        ## ACTOR NETWORK
        policy_dist = F.relu(self.actor_linear1(state))
        policy_dist = F.softmax(self.actor_linear2(policy_dist),dim=1)

        ## CRITIC NETWORK
        V_state = F.relu(self.critic_linear1(state))
        V_state = self.critic_linear2(V_state)

        return V_state,policy_dist

    def choose_action(self,state):

        ## FIND ACTION PROBABILITIES FOR CURRENT STATE 
        state = torch.tensor(state.reshape(1,-1),dtype=torch.float).to(DEVICE) # Convert state to tensor
        value, policy_dist = self.Actor_Critic.forward(state) # Probability for each action

        ## SAMPLE FROM PROBABILITY DISTRIBUTION
        dist = Categorical(policy_dist)
        action = dist.sample()

        return action.item(),value.item(),policy_dist # Check for valid data types


if __name__ == "__main__":
    env = gym.make("CartPole-v1")

    num_inputs = env.observation_space.shape[0]
    num_actions = env.action_space.n
    
    agent = A2C(num_inputs,num_actions,hidden_size=256)
    ep_score = []
    avg_ep_score = []
    entropy_term = 0

    for episode in range(EPISODES):

        log_probs = []
        values = []
        rewards = []
        
        done = False
        state = env.reset()

        while not done:

            if RENDER:
                env.render()

            V_state, policy_dist = agent.forward(state)
            V_state = V_state.item()
            policy_dist_np = policy_dist.cpu().detach().numpy()

            action = np.random.choice(num_actions,p=np.squeeze(policy_dist_np))
            log_prob = torch.log(policy_dist.squeeze()[action])
            entropy = -np.sum(np.mean(policy_dist_np)*np.log(policy_dist_np))

            next_state,reward,done,_ = env.step(action)

            rewards.append(reward)
            values.append(V_state)
            log_probs.append(log_prob)
            entropy_term += entropy

            if done:
                Q_val,_ = agent.forward(next_state)
                Q_val = Q_val.item()

                ep_score.append(np.sum(rewards))
                avg_ep_score.append(np.nanmean(ep_score[-10:]))

                if episode%100 == 0:
                    RENDER = True
                    print(f"EPISODE: {episode} Score: {np.sum(rewards)} MA_Reward: {np.nanmean(ep_score[-10:]):.2f}")
                else:
                    RENDER = False

            state = next_state

        ## COMPUTE Q VALUES
        Q_vals = np.zeros_like(values)
        for t in reversed(range(len(rewards))):
            Q_val = rewards[t] + GAMMA * Q_val
            Q_vals[t] = Q_val

        ## UPDATE ACTOR-CRITIC NETWORKS
        values = torch.tensor(values,dtype=torch.float).to(DEVICE)
        Q_vals = torch.tensor(Q_vals,dtype=torch.float).to(DEVICE)
        log_probs = torch.stack(log_probs)

        advantage = Q_vals - values
        actor_loss = (-log_probs*advantage).mean()
        critic_loss = 0.5*advantage.pow(2).mean()
        ac_loss = actor_loss + critic_loss + 0.001*entropy_term

        agent.optimizer.zero_grad()
        ac_loss.backward()
        agent.optimizer.step()



    # Plot results
    plt.plot(range(EPISODES),ep_score)
    plt.plot(range(EPISODES), avg_ep_score)
    plt.show()
        


