import numpy as np
import copy
import scipy.io
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from math import asin,pi,ceil

# Parent Evolutionary Strategy Class
# pepg and cma inherit common methods 
class ES:
    def __init__(self,gamma=0.8, n_rollout = 6):
        # ovveride this for necessary hyperparameters
        self.gamma, self.n_rollout = gamma, n_rollout


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
        r2test = np.zeros_like(r1)
        for k_quat in range(quat.shape[-1]):
            R = Rotation.from_quat(quat[:,k_quat])
            b3 = R.as_matrix()[:,2] # body z-axis

            r2[k_quat] = 0.5*(np.dot(b3, np.array([0,0,-1]))) + 0.5 # reward from orientation scaled 0-1
            r2test[k_quat] = asin(r2[k_quat])/pi + 0.5
            #if (r2[k_quat]>0.8) and (z[k_quat] > 0.8*h_ceiling):  # further incentivize when b3 is very close to -z axis
            #    r2[k_quat] = r2[k_quat]*5
            #if z[k_quat] < 0.3*h_ceiling:
            #    r2[k_quat] = 0
        #r = r1 + r2
        #print(r1,r2)

        # multioply elements of r1 and r2 for total reward at each step
        r = np.multiply(r1,r2)
        rplus = r1 + r2
        # testing different reward functions
        rtest = np.multiply(r1,r2test)
        #print(r)
        r_cum = np.zeros_like(r)
        r_cum1 = np.zeros_like(r1)
        r_cum2 = np.zeros_like(r2)
        r_cum2test = np.zeros_like(r2)
        r_cumtest = np.zeros_like(rtest)
        temp = 0
        temp1=0
        temp2=0
        temptest = 0
        temp2test =0
        tempplus = 0
        r_cumplus = np.zeros_like(rplus)
        for k_r in range(0,len(r)):
            tempplus = rplus[k_r] + self.gamma*tempplus
            r_cumplus = tempplus

            temp = r[k_r] + self.gamma*temp   # sum of r
            r_cum[k_r] = temp

            temp1 = r1[k_r] + self.gamma*temp1    # sum of just r1
            r_cum1[k_r] = temp1

            temp2 = r2[k_r] + self.gamma*temp2   # sum of just r2
            r_cum2[k_r] = temp2

            temptest = rtest[k_r] + self.gamma*temptest  # use different r2 (asin)
            r_cumtest[k_r] = temptest

            temp2test = r2test[k_r] + self.gamma*temp2test # use different r2 (asin)
            r_cum2test[k_r] = temp2test

        #print(r_cum1[-1],r_cum2[-1])
        #r_cum2[-1] = r_cum2[-1]*float(z[-1] > 1.2)
        if r_cum.size > 0:

            cum = r_cum1[-1]*(r_cum2[-1])
            print('r_cum1: %.3f \t r_cum2: %.3f' %(r1[-1],r2[-1]))
            
            print("[dot]     r1r2  = %.3f \t r1asin(r2) = %.3f" %(r_cum[-1],r_cumtest[-1]))
            print("[end only]r1r2 cum = %.3f \t r1asin(r2) = %.3f" %(cum/20.0,r_cum1[-1]*r_cum2test[-1]/20.0))
            print(r_cumplus)
            #print(r_cum[-1],cum,r_cum1[-1],r_cum2[-1])

            # Give extra reward if landed
            # these heights+oreintations tend to match up with 4,3,2,1 legs landing.
            # not perfect but does a good job rewarding landing more than other run
            '''if r1[-1] > 0.97 and r2[-1] > 0.98:
                return r_cum[-1] + 5
            elif r1[-1] > 0.95 and r2[-1] > 0.95:
                return r_cum[-1] + 4
            elif r1[-1] > 0.90 and r2[-1] > 0.8:
                return r_cum[-1] + 3
            elif r1[-1] > 0.9 and r2[-1] > 0.5:
                return r_cum[-1] + 2
            else:'''
            return   np.around(r_cum[-1],2) # float(z[-1]>1.2)*cum
            # max 1150 min -550 -> 0 - 1700
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
            alpha_mu (np.array): <u_learning rate
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
            if theta[1,k_n] > 0:
                theta[1,k_n] = -0.001
            '''if theta[2,k_n] < 0:
                theta[1,k_n] = 0.001
            if theta[3,k_n] < 0:
                theta[1,k_n] = 0'''

        return theta, epsilon

    def train(self, theta, reward, epsilon):
        reward_plus = reward[0:self.n_rollout]
        reward_minus = reward[self.n_rollout:]
        epsilon = epsilon
        b = self.get_baseline(span=3)

        m_reward = 26.0#400.0#3000#2300      # max reward
        reward_avg = np.mean(reward)
       
        ## Decaying Learning Rate:
        #self.alpha_mu = self. * 0.9
        #self.alpha_sigma = self.alpha_sigma * 0.9

        T = epsilon
        S = (T*T - self.sigma*self.sigma)/abs(self.sigma)
        r_T = (reward_plus - reward_minus) / (2*m_reward - reward_plus - reward_minus)
        r_S = ((reward_plus + reward_minus)/2 - b) / (m_reward - b)

         # Defeine learning rate scale depending on reward recieved
        lr_scale = 1.0 - reward_avg/m_reward # try squaring?

        explore_factor = 1.5 # determines how much faster sigma alpha decreases than mu alpha
        b2 = self.get_baseline(5)

        print(len(self.reward_history))
        print(self.reward_history.size)

        # burst exploration if algorithim gets stuck at local optimum
        if len(self.reward_history) > 20:
            if b2 < m_reward*0.7: # increase sigma alot of reward low
                lr_scale = 5.0*(1.0 - b2/m_reward)
            elif b2 < m_reward*(0.85): # increase sigma slightly of close to max
                lr_scale = 2.0*(1.0 - b2/m_reward)

        lr_sigma = (lr_scale**explore_factor)/explore_factor # adjust sigma alpha
        # update new learning rates
        # These only depend on the current episode
        self.alpha_mu = np.array([[lr_scale],[lr_scale] ])#,[lr_scale]])#0.95
        self.alpha_sigma = np.array([[lr_sigma],[lr_sigma] ])#,[lr_sigma]])  #0.95
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





    