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

    # def calculate_reward(self, _state, _h_ceiling):         # _state is size 13 x timesteps
    #     state = _state
    #     h_ceiling = _h_ceiling
    #     h_delta = 0.02     # 0.044

    #     z = state[2,:]
    #     quat = state[4:7,:]
    #     quat = np.append(quat, state[3,:][np.newaxis,:], axis=0)          # rearrange quat as scalar-last format used in scipy Rotation

    #     r1 = z / h_ceiling

    #     r2 = np.zeros_like(r1)
    #     for k_quat in range(quat.shape[-1]):
    #         R = Rotation.from_quat(quat[:,k_quat])
    #         b3 = R.as_matrix()[:,2]                         # body z-axis

    #         r2[k_quat] = np.dot(b3, np.array([0,0,-1]))
    #         if (r2[k_quat]>0.8) and (z[k_quat] > 0.8*h_ceiling):            # further incentivize when b3 is very close to -z axis
    #             r2[k_quat] = r2[k_quat]*5
    #         elif z[k_quat] < 0.5*h_ceiling:
    #             r2[k_quat] = 0
        
    #     r = r1 + r2
    #     r_cum = np.zeros_like(r)

    #     temp = 0
    #     for k_r in range(0,len(r)):
    #         temp = r[k_r] + self.gamma*temp
    #         r_cum[k_r] = temp

    #     if r_cum.size > 0:
    #         return r_cum[-1]
    #     else:
    #         return np.nan

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


    def calculate_reward(self, state, h_ceiling): # state is size 13 x timesteps
        # should be the same for each algorithm
        state = state
        h_ceiling = h_ceiling
        #h_delta = 0.02 # 0.044

        z = state[2,:]
        quat = state[4:7,:]
        # Rearrange quat as scalar-last format used in scipy Rotation
        # [qw,qx,qy,qz]' => quat = [qx,qy,qz,qw]'
        quat = np.append(quat, state[3,:][np.newaxis,:], axis=0)  # rearrange quat as scalar-last format used in scipy Rotation
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
    def __init__(self, alpha_mu, alpha_sigma, mu,sigma,gamma=0.95, n_rollout = 6):
        """[summary]

        Args:
            alpha_mu (np.array): mu_learning rate
            alpha_sigma (np.array): Sigma_learning rate
            mu (np.array): Parameter means
            sigma (np.array): Parameter Standard Deviations
            gamma (float, optional): Discount Factor. Defaults to 0.95.
            n_rollout (int, optional): Rollouts per episode. Defaults to 6.
        """        

        self.alpha_mu, self.alpha_sigma,  = alpha_mu, alpha_sigma
        self.gamma, self.n_rollout = gamma, n_rollout
        self.mu = mu
        self.sigma = sigma

        self.mu_history = copy.copy(self.mu)  # Creates another array of self.mu and attaches it to self.mu_history
        self.sigma_history = copy.copy(self.sigma)
        self.reward_history = np.array([0])

    def get_theta(self):
        zeros = np.zeros_like(self.mu)
        epsilon = np.random.normal(zeros, abs(self.sigma), [zeros.size, self.n_rollout]) # theta is size of  mu.size x n_runPerEp

        theta_plus = self.mu + epsilon
        theta_minus = self.mu - epsilon

        theta = np.append(theta_plus, theta_minus, axis=1)


        ## Caps values of RREV to be greater than zero and Gain to be neg so
        # the system doesnt fight itself learning which direction to rotate
        for k_n in range(2*self.n_rollout):
            if theta[0,k_n] < 0: # 
                theta[0,k_n] = 0.001
        '''if theta[1,k_n] > 0:
            theta[1,k_n] = -0.001
        if theta[2,k_n] < 0:
            theta[2,k_n] = 0.001'''


        return theta, epsilon

    def train(self, theta, reward, epsilon):
        reward_plus = reward[0:self.n_rollout]
        reward_minus = reward[self.n_rollout:]
        epsilon = epsilon
        b = self.get_baseline(span=3)

        m_reward = 21.0#400.0#3000#2300      # max reward
        reward_avg = np.mean(reward)
       
        ## Decaying Learning Rate:
        #self.alpha_mu = self. * 0.9
        #self.alpha_sigma = self.alpha_sigma * 0.9

        T = epsilon
        S = (T*T - self.sigma*self.sigma)/abs(self.sigma)
        r_T = (reward_plus - reward_minus) / (2*m_reward - reward_plus - reward_minus)
        r_S = ((reward_plus + reward_minus)/2 - b) / (m_reward - b)



        self.mu = self.mu + self.alpha_mu*(np.dot(T,r_T))
        self.sigma = self.sigma + self.alpha_sigma*np.dot(S,r_S)

        '''if self.mu[0,0] < 0:
            self.mu[0,0] = 0.1
        if self.mu[1,0] > 0:
            self.mu[1,0] = -0.1'''
        for k in range(self.sigma.size): #  If sigma oversteps negative then assume convergence
            if self.sigma[k] <= 0:
                self.sigma[k] = 0.05
        
        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)
        self.reward_history = np.append(self.reward_history, np.mean(reward))





class rlsysPEPGAgent_adaptive(rlsysPEPGAgent_reactive):
    def train(self, theta, reward, epsilon):
        reward_avg = np.mean(reward)
        if len(self.reward_history == 1):
            self.reward_history[0] = reward_avg
        else:
            self.reward_history = np.append(self.reward_history, reward_avg) 

        reward_plus = reward[0:self.n_rollout]
        reward_minus = reward[self.n_rollout:]
        epsilon = epsilon
        b = self.get_baseline(span=3)

        m_reward = 21.0#400.0#3000#2300      # max reward
        
       
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

        '''if self.mu[0,0] < 0:
            self.mu[0,0] = 0.1
        if self.mu[1,0] > 0:
            self.mu[1,0] = -0.1'''
        for k in range(self.sigma.size): #  If sigma oversteps negative then assume convergence
            if self.sigma[k] <= 0:
                self.sigma[k] = 0.05
        
        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)
        self.reward_history = np.append(self.reward_history, np.mean(reward))


class rlsysPEPGAgent_cov(rlsysPEPGAgent_reactive): # doesnt work breaks independance assumption at high sxy
    # covariance matrix becomes non semi positive definite (eigenvalue < 0)
    # need update rule based on multivariate gaussian
    def get_theta(self):
        mu = np.array([self.mu[0,0],self.mu[1,0]])
        zeros = np.zeros_like(mu)
        sigma = np.array([[self.sigma[0,0],self.sigma[2,0]],[self.sigma[2,0],self.sigma[1,0]]])

        epsilon = np.random.multivariate_normal(zeros,sigma,[1, self.n_rollout])
        epsilon = epsilon[0,:,:]
        epsilon = np.transpose(epsilon)
        print(epsilon)
        theta_plus = (self.mu + epsilon)
        theta_minus = (self.mu - epsilon)
        theta = np.append(theta_plus, theta_minus, axis=1)

        ## Caps values of RREV to be greater than zero and Gain to be neg so
        # the system doesnt fight itself learning which direction to rotate
        for k_n in range(2*self.n_rollout):
            #print(k_n)
            if theta[0,k_n] < 0: # 
                theta[0,k_n] = 0.001
            if theta[1,k_n] > 0:
                theta[1,k_n] = -0.001

        return theta, epsilon

    def train(self, theta, reward, epsilon):
        reward_plus = reward[0:self.n_rollout]
        reward_minus = reward[self.n_rollout:]
        epsilon = epsilon
        b = self.get_baseline(span=3)

        m_reward = 21.0 # max reward
        reward_avg = np.mean(reward)

        sigma = np.array([[self.sigma[0,0]],[self.sigma[1,0]]])

        T = epsilon
        S = (T*T - sigma*sigma)/abs(sigma)
        r_T = (reward_plus - reward_minus) / (2*m_reward - reward_plus - reward_minus)
        r_S = ((reward_plus + reward_minus)/2 - b) / (m_reward - b)

         # Defeine learning rate scale depending on reward recieved
        sigma = np.array([self.sigma[0,0],self.sigma[1,0]])
        lr_scale = min(1.0 - reward_avg/m_reward,1) # try squaring?

        explore_factor = 1.0 # determines how much faster sigma alpha decreases than mu alpha


        lr_sigma = (lr_scale**explore_factor)/explore_factor # adjust sigma alpha
        # update new learning rates
        # These only depend on the current episode
        self.alpha_mu = np.array([[lr_scale],[lr_scale] ])#,[lr_scale]])#0.95
        self.alpha_sigma = np.array([[lr_sigma],[lr_sigma] ])#,[lr_sigma]])  #0.95
        #print(self.alpha_mu,self.alpha_sigma)
        #print(np.dot(T,r_T),np.dot(S,r_S))
        self.mu = self.mu + self.alpha_mu*(np.dot(T,r_T))
        sigupdate = lr_sigma*np.dot(S,r_S)
        print(sigupdate)
        print(self.sigma)
        print(lr_sigma)
        print(np.dot(T,r_T))
        self.sigma[0,0] = self.sigma[0,0] + lr_sigma*sigupdate[0]
        self.sigma[1,0] = self.sigma[1,0] + lr_sigma*sigupdate[1]

        summary = np.concatenate((np.transpose(theta),reward),axis=1)
        summary = np.transpose(summary[summary[:,2].argsort()[::-1]])
        print(summary)
        summary_top = summary[:,0:self.n_rollout]
        r_ave_top = np.mean(summary_top[2,:])
        x_ave_top = np.mean(summary_top[0,:])
        y_ave_top = np.mean(summary_top[1,:])
        cov_top = np.sum(np.dot(summary_top[0,:]-x_ave_top,summary_top[1,:]-y_ave_top))
        print("cov = %.3f" %(cov_top))
        T_xy = (r_ave_top - b) / (m_reward - b)
        rrr = r_ave_top/m_reward
        lr_sigxy = min((r_ave_top/m_reward)**2,1)
        self.sigma[2,0] = (1-rrr)*self.sigma[2,0] + rrr*lr_sigxy*(cov_top - self.sigma[2,0])
        #self.sigma[2,0] = (1-rrr)*self.sigma[2,0] + rrr*lr_sigma*(cov_top-self.simga[2,0])/4 # maybe dont need
        C = np.array([[self.sigma[0,0],self.sigma[2,0]],[self.sigma[2,0],self.sigma[1,0]]])
        w,v = np.linalg.eig(C)
        print(w)
        for i in range(2):
            if w[i] < 0:
                w[i] = 0.01
        print(w)
        print(w)
        C2 = np.dot(np.dot(v,np.diag(w)),np.transpose(v))
        C = C2
        self.sigma[0,0] = C[0,0]
        self.sigma[1,0] = C[1,1]
        self.sigma[2,0] = C[1,0]
        v.dot(np.diag(w)).dot(v)
        print(C2)
        print(C)
        
        print(v)
        for k in range(self.sigma.size-1): #  If sigma oversteps negative then assume convergence
            if self.sigma[k] <= 0:
                self.sigma[k] = 0.05
        
        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)
        self.reward_history = np.append(self.reward_history, np.mean(reward))
    