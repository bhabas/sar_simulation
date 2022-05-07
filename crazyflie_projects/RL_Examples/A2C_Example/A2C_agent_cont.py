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
EPISODES = 2000
GAMMA = 0.99
NUM_STEPS = 50
LR = 0.0001
RENDER = True


class A2C(nn.Module):
    def __init__(self,num_inputs,num_actions,hidden_size,learning_rate=LR):
        super(A2C,self).__init__()

        self.num_inputs = num_inputs
        self.num_actions = num_actions

        self.base = nn.Sequential(
            nn.Linear(num_inputs,hidden_size),
            nn.ReLU(),
        )

        self.actor = nn.Sequential(
            nn.Linear(hidden_size,num_actions),
            nn.Softmax(dim=1),
        )

        self.critic = nn.Sequential(
            nn.Linear(hidden_size,1)
        )


        self.optimizer = optim.Adam(self.parameters(),lr=learning_rate)
        self.to(DEVICE)

    def forward(self,state):

        state = torch.tensor(state.reshape(1,-1),dtype=torch.float).to(DEVICE) # Convert state to tensor

        base_out = self.base(state)
        policy_dist = self.actor(base_out)
        V_state = self.critic(base_out)

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
    ep_score_list = []
    avg_ep_score_list = []
    entropy_term = 0

    for episode in range(EPISODES):

        ## RESET EPISODE
        done = False
        state = env.reset()
        ep_score = 0

        while not done:

            ## INITIALIZE EPISODE MINIBATCH
            log_probs = []
            V_states = []
            rewards = []

            for k in range(NUM_STEPS):

                if RENDER:
                    # env.render()
                    pass
                
                ## SELECT ACTION a_k USING ACTOR pi_theta
                V_state, policy_dist = agent.forward(state)
                V_state = V_state.item()
                policy_dist_np = policy_dist.cpu().detach().numpy()
                action = np.random.choice(num_actions,p=np.squeeze(policy_dist_np))

                log_prob = torch.log(policy_dist.squeeze()[action])
                entropy = -np.sum(np.mean(policy_dist_np)*np.log(policy_dist_np))

                ## PERFORM ACTION a_k AND OBSERVE NEXT STATE s_k+1 AND REWARD r_k+1
                next_state,reward,done,_ = env.step(action)

                ## STORE IN EPISODE MINIBATCH
                rewards.append(reward)
                V_states.append(V_state)
                log_probs.append(log_prob)
                entropy_term += entropy

                if done:
                    break

                state = next_state

            ## IF s_n IS NOT TERMINAL: SET R=V(s_n) WITH CRITIC, ELSE R = 0
            if not done:
                V_f,_ = agent.forward(next_state)
            else:
                V_f = 0

            ## CALCULATE DISCOUNTED RETURNS
            Rs = np.zeros_like(V_states)
            R = V_f
            for k in reversed(range(len(V_states))):
                R = rewards[k] + GAMMA*R
                Rs[k] = R

            

            ## UPDATE ACTOR-CRITIC NETWORKS
            V_states = torch.tensor(V_states,dtype=torch.float).to(DEVICE)
            Rs = torch.tensor(Rs,dtype=torch.float).to(DEVICE)
            log_probs = torch.stack(log_probs)

            advantage = Rs - V_states
            actor_loss = (-log_probs*advantage).mean()
            critic_loss = advantage.pow(2).mean()
            AC_Loss = actor_loss + critic_loss + 0.001*entropy_term*0.0

            agent.optimizer.zero_grad()
            AC_Loss.backward()
            agent.optimizer.step()

            ## UPDATE EPISODE SCORE
            ep_score += np.sum(rewards)

        ## APPEND EPISODE DATA
        ep_score_list.append(ep_score)
        avg_ep_score_list.append(np.nanmean(ep_score_list[-20:]))

        if episode%100 == 0:
            RENDER = True
            print(f"EPISODE: {episode} Score: {np.sum(rewards)} MA_Reward: {np.nanmean(ep_score_list[-20:]):.2f}")
        else:
            RENDER = False



    # PLOT RESULTS
    plt.plot(range(EPISODES),ep_score_list)
    plt.plot(range(EPISODES), avg_ep_score_list)
    plt.show()
        


