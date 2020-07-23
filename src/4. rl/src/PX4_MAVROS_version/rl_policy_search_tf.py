#!/usr/bin/env python

import numpy as np
import tensorflow as tf

class PolicySearch:
    def __init__(self, n_states, sigma, alpha, gamma=0.95):
        self.n_states = n_states
        #self.theta = np.ones(n_states+1)
        self.sigma = sigma
        
        self.alpha = alpha      # learning rate
        self.gamma = gamma      # discount factor

        self.theta = np.array([-1000.0, 500.0, 100.0])


    def _ini_nn(self, n_states):
        with tf.name_scope('inputs'):
            self.tf_states = tf.placeholder(tf.float32, shape=[None, n_states], name='states')

        layer_input = tf.layers.dense(
            inputs = self.tf_states,
            units = 10,
            activation = tf.nn.tanh,
            kernel_initializer = tf.random_normal_initializer(mean=0, stddev=10),
            bias_initializer = tf.constant_initializer(1),
            name = 'layer_input',
        )

        layer_mean = tf.layers.dense(
            inputs = layer_input,
            units = 1,
            activation = None,
            kernel_initializer = tf.random_normal_initializer(mean=0, stddev=10),
            bias_initializer = tf.constant_initializer(1),
            name = 'layer_mean'
        )

        self.layer_output = tf.
    
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
