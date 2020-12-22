import numpy as np
import copy
import scipy.io
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from math import asin,pi,ceil,floor
from rl_syspepg import ES

class rlEM_PEPGAgent(ES):
    def __init__(self,mu,sigma,gamma=0.95, n_rollout = 2):
        self.gamma, self.n_rollout = gamma, n_rollout
        self.mu = mu
        self.sigma = sigma

        self.n = 2*n_rollout
        self.d = len(self.mu)
        self.alpha_mu, self.alpha_sigma  = np.array([[0],[0]]),np.array([[0],[0]])
        self.mu_history = copy.copy(self.mu)  # Creates another array of self.mu and attaches it to self.mu_history
        self.sigma_history = copy.copy(self.sigma)
        self.reward_history = np.array([0])

    def get_theta(self):
        theta = np.zeros((self.d,self.n))
        for dim in range(0,self.d):
            theta[dim,:] = np.random.normal(self.mu[dim,0],self.sigma[dim,0],[1,self.n])

        #x = np.random.normal(self.mu[0,0],self.sigma[0,0],[1,self.n])
        #y = np.random.normal(self.mu[1,0],self.sigma[1,0],[1,self.n])
        #print(x)
        #print(y)
        print(theta)
        #theta = np.append(x,y,axis = 0)

        for k_n in range(self.n):
            if theta[0,k_n] < 0: # 
                theta[0,k_n] = 0.001 
            if theta[1,k_n] < 0:
               theta[1,k_n] = 0.001
            if theta[2,k_n] < 0:
                theta[2,k_n] = 0.001

        return theta , 0

    def train(self,theta,reward,epsilon):

        summary = np.concatenate((np.transpose(theta),reward),axis=1)
        summary = np.transpose(summary[summary[:,self.d].argsort()[::-1]])
        print(summary)

        k = floor(self.n/2)

        S_theta = (summary[0:self.d,0:k].dot(summary[self.d,0:k].reshape(k,1)))
        S_reward = np.sum(summary[self.d,0:k])

        S_diff = np.square(summary[0:self.d,0:k] - self.mu).dot(summary[self.d,0:k].reshape(k,1))

        
        self.mu = S_theta/(S_reward +0.001)
        self.sigma = np.sqrt(S_diff/(S_reward + 0.001))

        '''S_theta = np.dot(theta,reward)
        S_reward = np.sum(reward)
        S_diff = np.square(theta - self.mu).dot(reward)'''

class rlEM_AdaptiveAgent(rlEM_PEPGAgent):
    def train(self,theta,reward,epsilon):
        reward_mean = np.mean(reward) # try baseline
        print(len(self.reward_history))
        if len(self.reward_history == 1):
            self.reward_history[0] = reward_mean
        else:
            self.reward_history = np.append(self.reward_history, reward_mean)

        print(reward_mean)
        reward_b = np.zeros_like(reward)
        b = self.get_baseline(span=3)
        print(b)
        for count,r in enumerate(reward):
            reward_b[count] = max(0,r - b)

        summary = np.concatenate((theta.T,reward,reward_b),axis=1)
        summary = (summary[summary[:,self.d].argsort()[::-1]]).T
        print(summary)


        S_theta = summary[0:self.d,:]@summary[self.d+1,:].reshape(self.n,1)
        print("S theta = ", S_theta)
        S_reward = np.sum(summary[self.d+1,:])
        S_diff = np.square(summary[0:self.d,:] - self.mu)@(summary[self.d+1,:].reshape(self.n,1))
        print("S_sigma = ",S_diff)

        sii = np.sqrt(S_diff/(S_reward + 0.001))
        #sij = np.sign(S_ij)*np.sqrt(abs(S_ij)/(S_reward + 0.001)) # no need for sqrt
        #sij = S_ij/(S_reward + 0.001) # this should be correct update term from derivation

        self.mu =  S_theta/(S_reward +0.001)
        self.sigma = np.array([[sii[0,0]]])

        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)

class rlEM_MatrixAgent(rlEM_PEPGAgent): # ignore this
    def get_theta(self):
        # mueff format for np multivariate normal function
        mu = np.array([self.mu[0,0],self.mu[1,0]])


        sigma = np.array([[self.sigma[0,0],self.sigma[2,0]],[self.sigma[2,0],self.sigma[1,0]]])
        v,w = np.linalg.eig(sigma)
        print(v)
        print(w)

        theta = np.random.multivariate_normal(mu,sigma,[1, int(self.n_rollout*2)])
        theta = theta[0,:,:]
        theta = np.transpose(theta)
        #print(theta)
        #print(theta.shape)
        #print(type(theta))
        for k_n in range(self.n):
            if theta[0,k_n] < 0: # 
                theta[0,k_n] = 0.001
            if theta[1,k_n] > 0:
                theta[1,k_n] = -0.001
        return theta , 0

    def train(self,theta,reward,epsilon):
        summary = np.concatenate((np.transpose(theta),reward),axis=1)
        summary = np.transpose(summary[summary[:,self.d].argsort()[::-1]])
        print(summary)

        k = floor(self.n/2)
        print(summary[0:self.d,0:k])
        print(summary[self.d,0:k].reshape(k,1))


        S_theta = (summary[0:self.d,0:k].dot(summary[self.d,0:k].reshape(k,1)))
        S_reward = np.sum(summary[self.d,0:k])
        S_diff = np.square(summary[0:self.d,0:k] - self.mu).dot(summary[self.d,0:k].reshape(k,1))
        
        
        #S_matrix = 
        
        
        S_ij = ((summary[0,0:k] - self.mu[0,0])*(summary[1,0:k] - self.mu[1,0])).dot(summary[self.d,0:k].reshape(k,1))
        S_ij = S_ij[0]

        sii = np.sqrt(S_diff/(S_reward + 0.001))
        sij = np.sign(S_ij)*np.sqrt(abs(S_ij)/(S_reward + 0.001))

        self.mu = S_theta/(S_reward +0.001)
        self.sigma = np.array([ [sii[0,0]],[sii[1,0]],[sij]])




class rlEM_OutlierAgent(rlEM_PEPGAgent):
    def train(self,theta,reward,epsilon):

        summary = np.concatenate((np.transpose(theta),reward),axis=1)
        summary = np.transpose(summary[summary[:,self.d].argsort()[::-1]])
        print(summary)

        k = floor(self.n/2)

        # remove outliers
        S_theta = (summary[0:self.d,1:self.n-1].dot(summary[self.d,1:self.n-1].reshape(self.n-2,1)))
        S_reward = np.sum(summary[self.d,0:k])

        S_diff = np.square(summary[0:self.d,1:self.n-1] - self.mu).dot(summary[self.d,1:self.n-1].reshape(self.n-2,1))

        
        self.mu = S_theta/(S_reward +0.001)
        self.sigma = np.sqrt(S_diff/(S_reward + 0.001))

class rlEMsys_PEPGAgent(rlEM_PEPGAgent):
    def get_theta(self):

        zeros = np.zeros_like(self.mu)
        epsilon = np.random.normal(zeros, abs(self.sigma), [zeros.size, self.n_rollout]) # theta is size of  mu.size x n_runPerEp

        theta_plus = self.mu + epsilon
        theta_minus = self.mu - epsilon

        theta = np.append(theta_plus, theta_minus, axis=1)

        print(theta)
        #theta = np.append(x,y,axis = 0)

        for k_n in range(self.n):
            if theta[0,k_n] < 0: # 
                theta[0,k_n] = 0.001
            if theta[1,k_n] > 0:
                theta[1,k_n] = -0.001

        return theta , 0

    def train(self,theta,reward,epsilon):

        summary = np.concatenate((np.transpose(theta),reward),axis=1)
        summary = np.transpose(summary[summary[:,self.d].argsort()[::-1]])
        print(summary)

        k = floor(self.n/2)


        S_theta = (summary[0:self.d,0:k].dot(summary[self.d,0:k].reshape(k,1)))
        S_reward = np.sum(summary[self.d,0:k])

        S_diff = np.square(summary[0:self.d,0:k] - self.mu).dot(summary[self.d,0:k].reshape(k,1))

        
        self.mu = S_theta/(S_reward +0.001)
        self.sigma = np.sqrt(S_diff/(S_reward + 0.001))


class rlEM_PEPG_CovAgent(rlEM_PEPGAgent):
    def get_theta(self):
        # mueff format for np multivariate normal function
        mu = np.array([self.mu[0,0],self.mu[1,0]])

        sigma = np.array([[self.sigma[0,0]**2,self.sigma[2,0]],[self.sigma[2,0],self.sigma[1,0]**2]])
        v,w = np.linalg.eig(sigma)
        print(v)
        print(w)

        # normal

        theta = np.random.multivariate_normal(mu,sigma,[1, int(self.n_rollout*2)])
        theta = theta[0,:,:]
        theta = np.transpose(theta)

        # symmetric sampling
        '''zeros = np.array([0.0,0.0])
        epsilon = np.random.multivariate_normal(zeros,sigma,self.n_rollout)
        epsilon = np.transpose(epsilon)
        
        thetaplus = self.mu + epsilon
        thetaminus = self.mu - epsilon
        theta = np.append(thetaplus,thetaminus,axis=1)'''

        for k_n in range(self.n):
            if theta[0,k_n] < 0: # 
                theta[0,k_n] = 0.001
            if theta[1,k_n] > 0:
                theta[1,k_n] = -0.001
        return theta , 0

    def train(self,theta,reward,epsilon):
        summary = np.concatenate((np.transpose(theta),reward),axis=1)
        summary = np.transpose(summary[summary[:,self.d].argsort()[::-1]])
        print(summary)

        k = floor(self.n/2)

        S_theta = (summary[0:self.d,0:k].dot(summary[self.d,0:k].reshape(k,1)))
        S_reward = np.sum(summary[self.d,0:k])
        S_diff = np.square(summary[0:self.d,0:k] - self.mu).dot(summary[self.d,0:k].reshape(k,1))

        S_ij = ((summary[0,0:k] - self.mu[0,0])*(summary[1,0:k] - self.mu[1,0])).dot(summary[self.d,0:k].reshape(k,1))
        S_ij = S_ij[0]

        sii = np.sqrt(S_diff/(S_reward + 0.001))
        #sij = np.sign(S_ij)*np.sqrt(abs(S_ij)/(S_reward + 0.001)) # no need for sqrt
        sij = S_ij/(S_reward + 0.001) # this should be correct update term from derivation

        self.mu = S_theta/(S_reward +0.001)
        self.sigma = np.array([ [sii[0,0]],[sii[1,0]],[sij]])

class rlEM_AdaptiveCovAgent(rlEM_PEPG_CovAgent):
    def get_theta(self):
        mu = np.array([self.mu[0,0],self.mu[1,0]])

        sigma = np.array([[self.sigma[0,0]**2,self.sigma[2,0]],[self.sigma[2,0],self.sigma[1,0]**2]])
        v,w = np.linalg.eig(sigma)
        print(v)
        print(w)

        zeros = np.array([0.0,0.0])
        epsilon = np.random.multivariate_normal(zeros,sigma,self.n_rollout)
        epsilon = np.transpose(epsilon)
        
        thetaplus = self.mu + epsilon
        thetaminus = self.mu - epsilon
        theta = np.append(thetaplus,thetaminus,axis=1)

        for k_n in range(self.n):
            if theta[0,k_n] < 0: # 
                theta[0,k_n] = 0.001
            if theta[1,k_n] > 0:
                theta[1,k_n] = -0.001
        return theta , 0

    def train(self,theta,reward,epsilon):
        reward_mean = np.mean(reward) # try baseline
        print(len(self.reward_history))
        if len(self.reward_history == 1):
            self.reward_history[0] = reward_mean
        else:
            self.reward_history = np.append(self.reward_history, reward_mean)

        print(reward_mean)
        reward_b = np.zeros_like(reward)
        b = self.get_baseline(span=3)
        print(b)
        for count,r in enumerate(reward):
            reward_b[count] = max(0,r - b)

        summary = np.concatenate((theta.T,reward,reward_b),axis=1)
        summary = (summary[summary[:,self.d].argsort()[::-1]]).T
        print(summary)


        S_theta = summary[0:self.d,:]@summary[self.d+1,:].reshape(self.n,1)
        print("S theta = ", S_theta)
        S_reward = np.sum(summary[self.d+1,:])
        S_diff = np.square(summary[0:self.d,:] - self.mu)@(summary[self.d+1,:].reshape(self.n,1))
        print("S_sigma = ",S_diff)
        S_ij = ((summary[0,:] - self.mu[0,0])*(summary[1,:] - self.mu[1,0]))@(summary[self.d+1,:].reshape(self.n,1))
        S_ij = S_ij[0]

        sii = np.sqrt(S_diff/(S_reward + 0.001))
        #sij = np.sign(S_ij)*np.sqrt(abs(S_ij)/(S_reward + 0.001)) # no need for sqrt
        sij = S_ij/(S_reward + 0.001) # this should be correct update term from derivation

        self.mu =  S_theta/(S_reward +0.001)
        self.sigma = np.array([ [sii[0,0]],[sii[1,0]],[sij]])

        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)

        #print(self.mu)
        #print(self.sigma)

class rlEM_AdaptiveCovAgent3D(rlEM_PEPG_CovAgent):
    def get_theta(self):
        mu = np.array([self.mu[0,0],self.mu[1,0],self.mu[2,0]])
        # s = sxx syy szz sxy sxz syz
        sigma = np.array([[self.sigma[0,0]**2,self.sigma[3,0],self.sigma[4,0]],[self.sigma[3,0],self.sigma[1,0]**2,self.sigma[5,0]],[self.sigma[4,0],self.sigma[5,0],self.sigma[2,0]**2]   ])
        #sigma = np.array([[self.sigma[0,0]**2,self.sigma[2,0]],[self.sigma[2,0],self.sigma[1,0]**2]])
        v,w = np.linalg.eig(sigma)
        print(v)
        print(w)

        zeros = np.array([0.0,0.0,0.0])
        epsilon = np.random.multivariate_normal(zeros,sigma,self.n_rollout)
        epsilon = np.transpose(epsilon)
        
        thetaplus = self.mu + epsilon
        thetaminus = self.mu - epsilon
        theta = np.append(thetaplus,thetaminus,axis=1)

        for k_n in range(self.n):
            if theta[0,k_n] < 0: # 
                theta[0,k_n] = 0.001
            if theta[1,k_n] > 0:
                theta[1,k_n] = -0.001
            if theta[2,k_n] < 0:
                theta[2,k_n] = 0.001
        return theta , 0

    def train(self,theta,reward,epsilon):
        reward_mean = np.mean(reward) # try baseline
        print(len(self.reward_history))
        if len(self.reward_history == 1):
            self.reward_history[0] = reward_mean
        else:
            self.reward_history = np.append(self.reward_history, reward_mean)

        print(reward_mean)
        reward_b = np.zeros_like(reward)
        b = self.get_baseline(span=3)
        print(b)
        for count,r in enumerate(reward):
            reward_b[count] = max(0,r - b)

        summary = np.concatenate((theta.T,reward,reward_b),axis=1)
        summary = (summary[summary[:,self.d].argsort()[::-1]]).T
        print(summary)


        S_theta = summary[0:self.d,:]@summary[self.d+1,:].reshape(self.n,1)
        print("S theta = ", S_theta)
        S_reward = np.sum(summary[self.d+1,:])
        S_diff = np.square(summary[0:self.d,:] - self.mu)@(summary[self.d+1,:].reshape(self.n,1))
        print("S_sigma = ",S_diff)
        S_xy = ((summary[0,:] - self.mu[0,0])*(summary[1,:] - self.mu[1,0]))@(summary[self.d+1,:].reshape(self.n,1))
        S_xz = ((summary[0,:] - self.mu[0,0])*(summary[2,:] - self.mu[2,0]))@(summary[self.d+1,:].reshape(self.n,1))
        S_yz = ((summary[1,:] - self.mu[1,0])*(summary[2,:] - self.mu[2,0]))@(summary[self.d+1,:].reshape(self.n,1))
        S_xy = S_xy[0]
        S_xz = S_xz[0]
        S_yz = S_yz[0]


        sii = np.sqrt(S_diff/(S_reward + 0.001))
        #sij = np.sign(S_ij)*np.sqrt(abs(S_ij)/(S_reward + 0.001)) # no need for sqrt
        sxy = S_xy/(S_reward + 0.001) # this should be correct update term from derivation
        sxz = S_xz/(S_reward + 0.001)
        syz = S_yz/(S_reward + 0.001)
        self.mu =  S_theta/(S_reward +0.001)
        self.sigma = np.array([ [sii[0,0]],[sii[1,0]],[sii[2,0]],[sxy],[sxz],[syz]])

        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)

        print(self.mu)
        print(self.sigma)
if __name__ == "__main__":
    mu = np.array([[1.0],[-4.0]])
    sigma = np.array([[0.1],[1.5],[0.0]])
    #agent = rlEM_MatrixAgent(mu,sigma)
    agent = rlEM_AdaptiveCovAgent(mu,sigma,n_rollout=3)
    theta,epsilon = agent.get_theta()
    reward = np.array([[1.0],[2.0],[0.3],[4.0],[2.5],[1.2] ])
    agent.train(theta,reward,0)

