#!/usr/bin/env python

import numpy as np
import scipy.io
import matplotlib.pyplot as plt

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


class DMP:
    def __init__(self, file_name):
        self.y_demo, self.params, self.basisFcnParams = self.read_dmp_params(file_name)

    def transformationSystem(self, duration, y0, tau, goal):
        alpha_y = self.params['alpha_y']
        beta_y = self.params['beta_y']
        alpha_x = self.params['alpha_x']
        basisCenters = self.basisFcnParams['basisFcnCenters']
        basisWidths = self.basisFcnParams['basisFcnWidths']
        w = self.basisFcnParams['basisFcnWeights']
        g = goal
        dt = 1.0/5000

        t = np.arange(0, duration, dt)
        nStep = t.size
        y = np.zeros([3, nStep+1])
        y[0,0] = y0[0]
        y[1,0] = y0[1]
        
        x = self.solveCanonicalSystem(t, tau, alpha_x)
        nBasisFcn = len(basisCenters)
        Phi = np.zeros([nBasisFcn, nStep])
        for k_nBasisFcn in range(nBasisFcn):
            Phi[k_nBasisFcn,:] = self.basisFunction(basisCenters[k_nBasisFcn], basisWidths[k_nBasisFcn], x)
        
        f = np.dot(w, Phi) / np.sum(Phi,axis=0) * x * (g - y[0,0])
        
        for k_step in range(nStep):
            y[2,k_step] = ( alpha_y*(beta_y*(g-y[0,k_step]) - tau*y[1,k_step]) + f[k_step] )/tau**2
            y[1,k_step+1] = y[1,k_step] + dt*y[2,k_step]
            y[0,k_step+1] = y[0,k_step] + dt*y[1,k_step]
        
        y = y[:,0:-1]
        
        # plt.figure()
        # plt.plot(t,y[0,:])
        # plt.show()

        t_y = np.vstack( (t,y) )

        return t_y

    def solveCanonicalSystem(self, t, tau, alpha_x):
        x = np.exp(-alpha_x/tau * t)
        return x

    def basisFunction(self, center, width, x):
        c = center
        sigma = width
        Phi = np.exp(-1/(2*sigma**2) * (x-c)**2)
        return Phi

    def read_dmp_params(self, file_name):
        file_name2 = '/home/pan/catkin_ws/src/robot landing/4. rl/src/' + file_name + '.mat'
        
        mat = scipy.io.loadmat(file_name2)

        y_demo = mat[file_name]['y_demo'][0][0]
        tau = mat[file_name]['tau'][0][0][0][0]
        alpha_y = mat[file_name]['alpha_y'][0][0][0][0]
        beta_y = mat[file_name]['beta_y'][0][0][0][0]
        alpha_x = mat[file_name]['alpha_x'][0][0][0][0]
        basisFcnCenters = mat[file_name]['basisFcnCenters'][0][0][0]
        basisFcnWidths = mat[file_name]['basisFcnWidths'][0][0][0]
        basisFcnWeights = mat[file_name]['basisFcnWeights'][0][0][0]

        params = {'tau': tau, 'alpha_y': alpha_y, 'beta_y': beta_y, 'alpha_x': alpha_x}
        basisFcnParams = {'basisFcnCenters': basisFcnCenters, 'basisFcnWidths': basisFcnWidths, 'basisFcnWeights': basisFcnWeights}

        return y_demo, params, basisFcnParams
