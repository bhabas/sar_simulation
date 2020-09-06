import numpy as np
import copy
import scipy.io
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

class rlsysPEPGAgent_reactive:
    def __init__(self, _alpha_mu, _alpha_sigma, _gamma=0.95, _n_rollout = 6):
        self.alpha_mu_, self.alpha_sigma_,  = _alpha_mu, _alpha_sigma
        self.gamma_, self.n_rollout_ = _gamma, _n_rollout

        self.mu_ = np.array([[5.0], [-10.0]])   # Initial estimates of mu: size (2 x 1)
        self.sigma_ = np.array([[1], [1]])      # Initial estimates of sigma: size (2 x 1)
        self.mu_history_ = copy.copy(self.mu_)  # Creates another array of self.mu_ and attaches it to self.mu_history_
        self.sigma_history_ = copy.copy(self.sigma_)
        self.reward_history_ = np.array([0])

    def calculate_reward(self, _state, _h_ceiling): # _state is size 13 x timesteps
            state = _state
            h_ceiling = _h_ceiling
            h_delta = 0.02 # 0.044

            z = state[2,:]
            quat = state[4:7,:]
            # Rearrange quat as scalar-last format used in scipy Rotation
            # [qw,qx,qy,qz]' => quat = [qx,qy,qz,qw]'
            quat = np.append(quat, state[3,:][np.newaxis,:], axis=0)  # rearrange quat as scalar-last format used in scipy Rotation

            r1 = z / h_ceiling

            r2 = np.zeros_like(r1)
            for k_quat in range(quat.shape[-1]):
                R = Rotation.from_quat(quat[:,k_quat])
                b3 = R.as_matrix()[:,2] # body z-axis

                r2[k_quat] = np.dot(b3, np.array([0,0,-1]))
                if (r2[k_quat]>0.8) and (z[k_quat] > 0.8*h_ceiling):  # further incentivize when b3 is very close to -z axis
                    r2[k_quat] = r2[k_quat]*5
                elif z[k_quat] < 0.5*h_ceiling:
                    r2[k_quat] = 0
            
            r = r1 + r2
            r_cum = np.zeros_like(r)

            temp = 0
            for k_r in range(0,len(r)):
                temp = r[k_r] + self.gamma_*temp
                r_cum[k_r] = temp

            if r_cum.size > 0:
                return r_cum[-1]
            else:
                return np.nan

    def get_theta(self):
        zeros = np.zeros_like(self.mu_)
        epsilon = np.random.normal(zeros, self.sigma_, [zeros.size, self.n_rollout_]) # theta is size of  mu.size x n_runPerEp

        theta_plus = self.mu_ + epsilon
        theta_minus = self.mu_ - epsilon

        theta = np.append(theta_plus, theta_minus, axis=1)


        ## Caps values of RREV to be greater than zero and Gain to be neg so
        # the system doesnt fight itself learning which direction to rotate
        for k_n in range(2*self.n_rollout_):
            if theta[0,k_n] < 0:
                theta[0,k_n] = 0.001
            if theta[1,k_n] > 0:
                theta[1,k_n] = 0

        return theta, epsilon

    def get_baseline(self, _span):
        if self.reward_history_.size < _span:
            b = np.mean(self.reward_history_)
        else:
            b = np.mean(self.reward_history_[-_span:])

        return b


    def train(self, _theta, _reward, _epsilon):
        reward_plus = _reward[0:self.n_rollout_]
        reward_minus = _reward[self.n_rollout_:]
        epsilon = _epsilon
        b = self.get_baseline(_span=3)
        m_reward = 125      # max reward

        ## Decaying Learning Rate:
        #self.alpha_mu_ = self.alpha_mu_ * 0.9
        #self.alpha_sigma_ = self.alpha_sigma_ * 0.9

        T = epsilon
        S = (T*T - self.sigma_*self.sigma_)/self.sigma_
        r_T = (reward_plus - reward_minus) / (2*m_reward - reward_plus - reward_minus)
        r_S = ((reward_plus + reward_minus)/2 - b) / (m_reward - b)
        
        self.mu_ = self.mu_ + self.alpha_mu_*np.dot(T,r_T)
        self.sigma_ = self.sigma_ + self.alpha_sigma_*np.dot(S,r_S)

        for k in range(self.sigma_.size): #  If sigma oversteps negative then assume convergence
            if self.sigma_[k] <= 0:
                self.sigma_[k] = 0.001
        
        self.mu_history_ = np.append(self.mu_history_, self.mu_, axis=1)
        self.sigma_history_ = np.append(self.sigma_history_, self.sigma_, axis=1)
        self.reward_history_ = np.append(self.reward_history_, np.mean(_reward))





    