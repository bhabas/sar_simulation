#!/usr/bin/env python

import numpy as np

class PolicySearch:
    def __init__(self, n_states, sigma, alpha, gamma=0.95):
        self.n_states = n_states
        #self.theta = np.ones(n_states+1)
        self.sigma = sigma
        
        self.alpha = alpha      # learning rate
        self.gamma = gamma      # discount factor

        self.theta = np.array([-1000.0, 500.0, 100.0])


    def learning(self, state, action, reward):
        state = np.hstack([state,1])
        
        delta_theta = np.dot(state,self.theta)
        delta_theta = action - delta_theta
        delta_theta = delta_theta*reward
        delta_theta = delta_theta*state
        delta_theta = delta_theta/np.square(self.sigma)
        delta_theta = delta_theta/len(state)

        self.theta = self.theta + self.alpha*delta_theta

        return self.theta


    def choose_action(self, state):
        state = np.hstack((state,1))
        action = np.dot(self.theta, state)
        action += np.random.normal(0, self.sigma, 1)
        return action


    def chose_action_random(self):
        return np.random.normal(1000, self.sigma, 1)
