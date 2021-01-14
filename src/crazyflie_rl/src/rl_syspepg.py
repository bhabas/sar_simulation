import numpy as np
import copy
import scipy.io
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from math import asin,pi,ceil,floor

# Parent Evolutionary Strategy Class
# pepg and cma inherit common methods 

# this needs to be reorganized much better, still lots of repetition
# maybe just use one class for each algorithm with options?
class ES:
    def __init__(self,gamma=0.8, n_rollout = 6):
        # ovveride this for necessary hyperparameters
        self.gamma, self.n_rollout = gamma, n_rollout

    
    def calculate_reward2(self,state,h_ceiling):
        h_delta = 0.02 
        e3 = np.array([0,0,1])
        
        ## R1 Calc
        z_hist = state[2,:]
        r1 = np.max(z_hist/h_ceiling)*10

        ## R2 Calc
        qw,qx,qy,qz = state[3:7,:]
        quat_hist = np.c_[qx,qy,qz,qw] # Create an array of quat objects in [qx,qy,qz,qw] format

        r2_vec = np.zeros_like(qw)
        for ii,quat in enumerate(quat_hist):
            R = Rotation.from_quat(quat)
            b3 = R.as_matrix()[:,2]                 # Body Z-axis in global frame
            r2_vec[ii] = 0.5*np.dot(b3,-e3) + 0.5   # Scale orientation reward to be from [0-1]


        ## R3 Calc
        omega_x,omega_y,omega_z = state[10:13,:]
        r3_vec = np.zeros_like(omega_y)
        for ii,omega_y in enumerate(omega_y):
            r3_vec[ii] = np.exp(-1/4*np.abs(omega_y))   # e^-1/4*|omega_y|


        r23 = r2_vec*10 + r3_vec*5
        r_prev = 0
        r_cum = np.zeros_like(omega_x)
        for ii,r in enumerate(r23):
            r_cum[ii] = r + 0.01*(r-r_prev)
            r_prev = r

        reward = r1 + r_cum[-1]
        return reward


    def calculate_reward(self, state_hist, h_ceiling): # state_hist is size 14 x timesteps
        # should be the same for each algorithm



        z = state_hist[3,:]
        quat_xyz = state_hist[5:8,:]
        quat_w = state_hist[4,:][np.newaxis,:]
        # Rearrange quat as scalar-last format used in scipy Rotation
        # [qw,qx,qy,qz]' => quat = [qx,qy,qz,qw]'
        quat = np.append(quat_xyz, quat_w, axis=0)  # rearrange quat as scalar-last format used in scipy Rotation
        r1 = z / h_ceiling # reward from hieght scaled 0-1

        r2 = np.zeros_like(r1)

        for k_quat in range(quat.shape[-1]):
            R = Rotation.from_quat(quat[:,k_quat])
            b3 = R.as_matrix()[:,2] # body z-axis
            r2[k_quat] = 0.5*(np.dot(b3, np.array([0,0,-1]))) + 0.5 # reward from orientation scaled 0-1

        # multiply elements of r1 and r2 for total reward at each step
        r = np.multiply(r1,r2)
        r_cum = np.zeros_like(r)

        temp = 0
        for k_r in range(0,len(r)):
            temp = r[k_r] + self.gamma*temp   # sum of r
            r_cum[k_r] = temp

        if r_cum.size > 0:
            return np.around(r_cum[-1],2) # float(z[-1]>1.2)*cum

        else:
            return np.nan

    def get_baseline(self, span):
        # should be the same
        if self.reward_history.size < span:
            b = np.mean(self.reward_history)
        else:
            b = np.mean(self.reward_history[-span:])

        return b

class rlsysPEPGAgent_reactive(ES):
    def __init__(self, alpha_mu, alpha_sigma, mu,sigma,gamma=0.95, n_rollouts = 6):
        self.gamma = gamma
        self.n_rollouts = n_rollouts
        self.agent_type = 'PEPG_reactive'

        self.alpha_mu =  alpha_mu
        self.alpha_sigma = alpha_sigma
        
        self.mu = mu
        self.sigma = sigma

        self.mu_history = copy.copy(self.mu)  # Creates another array of self.mu and attaches it to self.mu_history
        self.sigma_history = copy.copy(self.sigma)
        self.reward_history = np.array([0])

    def get_theta(self):
        zeros = np.zeros_like(self.mu)
        epsilon = np.random.normal(zeros, abs(self.sigma), [zeros.size, self.n_rollouts//2]) # theta is size of  mu.size x n_runPerEp

        theta_plus = self.mu + epsilon
        theta_minus = self.mu - epsilon

        theta = np.append(theta_plus, theta_minus, axis=1)


        ## Caps values of RREV to be greater than zero and Gain to be neg so
        # the system doesnt fight itself learning which direction to rotate
        theta[theta<=0] = 0.001


        return theta, epsilon

    def train(self, theta, reward, epsilon):
        reward_plus = reward[0:self.n_rollouts//2]
        reward_minus = reward[self.n_rollouts//2:]
        epsilon = epsilon
        b = self.get_baseline(span=3)

        m_reward = 21.0 # max reward
  
        ## Decaying Learning Rate:
        #self.alpha_mu = self. * 0.9
        #self.alpha_sigma = self.alpha_sigma * 0.9

        T = epsilon
        S = (T*T - self.sigma*self.sigma)/abs(self.sigma)
        r_T = (reward_plus - reward_minus) / (2*m_reward - reward_plus - reward_minus)
        r_S = ((reward_plus + reward_minus)/2 - b) / (m_reward - b)



        self.mu = self.mu + self.alpha_mu*(np.dot(T,r_T))
        self.sigma = self.sigma + self.alpha_sigma*np.dot(S,r_S)

        self.sigma[self.sigma<=0] = 0.05 #  If sigma oversteps negative then assume convergence
        
        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)
        self.reward_history = np.append(self.reward_history, np.mean(reward))





class rlsysPEPGAgent_adaptive(rlsysPEPGAgent_reactive):
    def __init__(self):
        self.agent_type = 'PEPG_adaptive'
    def train(self, theta, reward, epsilon):
        reward_avg = np.mean(reward)
        if len(self.reward_history == 1):
            self.reward_history[0] = reward_avg
        else:
            self.reward_history = np.append(self.reward_history, reward_avg) 

        reward_plus = reward[0:self.n_rollouts//2]
        reward_minus = reward[self.n_rollouts//2:]
        epsilon = epsilon
        b = self.get_baseline(span=3)

        m_reward = 21.0 # max reward
        
       
        ## Decaying Learning Rate:
        #self.alpha_mu = self. * 0.9
        #self.alpha_sigma = self.alpha_sigma * 0.9

        T = epsilon
        S = (T*T - self.sigma*self.sigma)/abs(self.sigma)
        r_T = (reward_plus - reward_minus) / (2*m_reward - reward_plus - reward_minus)
        r_S = ((reward_plus + reward_minus)/2 - b) / (m_reward - b)

         # Defeine learning rate scale depending on reward recieved
        #lr_scale = 1.0 - reward_avg/m_reward # try squaring?
        lr_scale = 1.0 - b/m_reward
        explore_factor = 1.0 # determines how much faster sigma alpha decreases than mu alpha
        b2 = self.get_baseline(5)

        print(len(self.reward_history))
        print(self.reward_history.size)

        # burst exploration if algorithim gets stuck at local optimum
        # CAUSES OVERSHOOTING
        if len(self.reward_history) > 20:
            if b2 < m_reward*0.7: # increase sigma alot of reward low
                lr_scale = 5.0*(1.0 - b2/m_reward)
            elif b2 < m_reward*(0.85): # increase sigma slightly of close to max
                lr_scale = 2.0*(1.0 - b2/m_reward)

        lr_sigma = (lr_scale**explore_factor)/explore_factor # adjust sigma alpha
        # update new learning rates
        # These only depend on the current episode
        self.alpha_mu = np.array([[lr_scale],[lr_scale] ,[lr_scale]])#0.95
        self.alpha_sigma = np.array([[lr_sigma],[lr_sigma] ,[lr_sigma]])  #0.95
        print(self.alpha_mu,self.alpha_sigma)
        print(np.dot(T,r_T),np.dot(S,r_S))
        self.mu = self.mu + self.alpha_mu*(np.dot(T,r_T))
        self.sigma = self.sigma + self.alpha_sigma*np.dot(S,r_S)

        self.sigma[self.sigma<=0] = 0.05 #  If sigma oversteps negative then assume convergence
        
        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)
        self.reward_history = np.append(self.reward_history, np.mean(reward))


