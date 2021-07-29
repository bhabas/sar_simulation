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
    def __init__(self, n_rollout = 6):
        # ovveride this for necessary hyperparameters
        self.n_rollout = n_rollout


    def calcReward_pureLanding(self,env):

        ## CALC R_3 FROM MAXIMUM HEIGHT ACHIEVED
        R_1 = (env.z_max/env.h_ceiling)*10

        ## CALC R2 FROM THE MAXIMUM PITCH ANGLE (CONVERT TO IMPACT ANGLE?)
        if -170 < np.min(env.pitch_max) <= 0:
            R_2 = 10*(-1/160*np.min(env.pitch_max))      
        elif -330 <= np.min(env.pitch_max) <= -170:
            R_2 = 10
        else:
            R_2 = 0



        ## CALC R_3 FROM FINAL ORIENTATION AT END OF ROLLOUT
        Rot = Rotation.from_quat([env.orientation_q[1],env.orientation_q[2],env.orientation_q[3],env.orientation_q[0]])
        b3 = Rot.as_matrix()[:,2]
        R_3 = np.exp(np.dot(b3, np.array([0,0,-1]))-1)*10 # (0.135 < e^(x-1) <= 1.0) | (-1 < x <= 1)


        if len(env.pad_contacts) >= 3 and env.body_contact == False:
            R_4 = 100
            
        elif len(env.pad_contacts) >= 3 and env.body_contact == True:
            R_4 = 10

        elif len(env.pad_contacts) == 2:
            R_4 = 5
        
        elif len(env.pad_contacts) == 1:
            R_4 = 2.5
        
        else:
            R_4 = 0.0


        R_total = R_1 + R_2 + R_4 + 0.001
        # print(f"Reward: r_c: {r_contact:.3f} | r_theta: {r_theta:.3f} | r_h: {r_h:.3f} | Pitch Max: {env.pitch_max:.2f}")
        return R_total



    def calcReward_Impact(self,env):

        ## CALC R_1 FROM MAXIMUM HEIGHT ACHIEVED
        R_1 = (env.z_max/env.h_ceiling)

        ## CALC R2 FROM THE IMPACT ANGLE

        if env.quat_impact.all() == 0.0: # Check if valid quat
            env.quat_impact = [0,0,0,1]

        R = Rotation.from_quat(env.quat_impact)
        eul_impact_arr = R.as_euler('YZX', degrees=True)
        eul_y_impact = eul_impact_arr[0]
        # Center axis [theta_2] is limited to +- 90deg while [theta_1 & theta_3] can rotate to 180deg 
        # https://graphics.fandom.com/wiki/Conversion_between_quaternions_and_Euler_angles
        # https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/Quaternions.pdf
        

        if -180 <= eul_y_impact <= -90:
            R_2 = 1.0
        elif -90 < eul_y_impact <= 0:
            R_2 = -1/90*eul_y_impact
        elif 0 < eul_y_impact <= 90:
            R_2 = 1/90*eul_y_impact
        elif 90 < eul_y_impact <= 180:
            R_2 = 1.0
        else:
            R_2 = 0


        if env.impact_flag == True and env.body_contact == False:
            ## CALC R_3 FROM FINAL ORIENTATION AT END OF ROLLOUT
            Rot = Rotation.from_quat([env.orientation_q[1],env.orientation_q[2],env.orientation_q[3],env.orientation_q[0]])
            b3 = Rot.as_matrix()[:,2]
            R_3 = np.exp(np.dot(b3, np.array([0,0,-1]))-1)*150 # (0.135 < e^(x-1) <= 1.0) | (-1 < x <= 1)

        else:
            R_3 = 0
        # ## CALC R_4 FROM NUMBER OF LEGS CONNECT
        # if env.impact_flag == True: # The pad connection callback is weird w/o impact so only check in this case

        #     if len(env.pad_contacts) >= 3: 
        #         if env.body_contact == False:
        #             R_4 = 150
        #         else:
        #             R_4 = 100
                
        #     elif len(env.pad_contacts) == 2: 
        #         if env.body_contact == False:
        #             R_4 = 50
        #         else:
        #             R_4 = 25
                    
        #     elif len(env.pad_contacts) == 1:
        #         R_4 = 10
            
        #     else:
        #         R_4 = 0.0
        # else:
        #     R_4 = 0.0


        R_total = R_1*10 + R_2*10 + R_3 + 0.001
        # print(f"Reward: r_c: {r_contact:.3f} | r_theta: {r_theta:.3f} | r_h: {r_h:.3f} | Pitch Max: {env.pitch_max:.2f}")
        return R_total

    def get_baseline(self, span):
        # should be the same
        if self.reward_history.size < span:
            b = np.mean(self.reward_history)
        else:
            b = np.mean(self.reward_history[-span:])

        return b

class rlsysPEPGAgent_reactive(ES):
    def __init__(self, alpha_mu, alpha_sigma, mu,sigma, n_rollouts = 6):
        self.n_rollouts = n_rollouts
        self.agent_type = 'SyS-PEPG_reactive'

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
        self.agent_type = 'SyS-PEPG_adaptive'
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


