import numpy as np
import copy
import scipy.io
from scipy.spatial.transform import Rotation
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


class rlPEPGAgent:
    def __init__(self, _alpha_mu, _alpha_sigma, _gamma=0.95, _n_runPerEp = 6):
        self.alpha_mu_, self.alpha_sigma_, self.gamma_ = _alpha_mu, _alpha_sigma, _gamma
        self.n_runPerEp_ = _n_runPerEp

        #self.mu_ = np.array([[13.0], [36.2], [2.0], [1], [1.0]])           # mu    is size of  5 x 1
        self.mu_ = np.array([[20.0], [36.2], [2.0], [1.0], [1.0]])           # mu    is size of  5 x 1
        self.sigma_ = np.array([[0.5], [1.0], [0.5], [0.5], [1.0]])         # sigma is size of  5 x 1
        self.mu_normalize_factor_ = np.array([[50.0], [50], [10.0], [np.pi], [1.0]])
        self.sigma__normalize_factor_ = np.array([[1.0], [1.0], [1.0], [1.0], [1.0]])
        self.mu_history_ = copy.copy(self.mu_)
        self.sigma_history_ = copy.copy(self.sigma_)
        self.reward_history_ = np.array([0])

    def train(self, _theta, _reward):
        theta = _theta
        b = self.get_baseline(_span=3)

        T = theta - self.mu_
        S = (T*T - self.sigma_*self.sigma_)/self.sigma_
        r = _reward - b
        
        self.mu_ = self.mu_ + self.alpha_mu_*np.dot(T,r)
        self.sigma_ = self.sigma_ + self.alpha_sigma_*np.dot(S,r)

        if self.mu_[0] < 7:
            self.mu_[0] = 7
        if self.mu_[0] > 40:
            self.mu_[0] = 40
        if self.mu_[1] < 5:
            self.mu_[1] = 5
        if self.mu_[2] < 0.5:
            self.mu_[2] = 0.5
        if self.mu_[2] > 10:
            self.mu_[2] = 10
        if self.mu_[3] < 0.1:
            self.mu_[3] = 0.1
        if self.mu_[4] < 1:
            self.mu_[4] = 1
        self.mu_[4] = 0
        self.sigma_[4] = 0.01

        for k in range(self.sigma_.size):
            if self.sigma_[k] <= 0:
                self.sigma_[k] = 0.001
        
        self.mu_history_ = np.append(self.mu_history_, self.mu_, axis=1)
        self.sigma_history_ = np.append(self.sigma_history_, self.sigma_, axis=1)
        self.reward_history_ = np.append(self.reward_history_, np.mean(_reward))

    def get_theta(self):
        theta = np.random.normal(self.mu_, self.sigma_, [self.mu_.size, self.n_runPerEp_])     # theta is size of  mu.size x n_runPerEp

        for k_n in range(self.n_runPerEp_):
            if theta[0,k_n] < 2:
                theta[0,k_n] = 2
            if theta[0,k_n] > 40:
                theta[0,k_n] = 40
            if theta[1,k_n] < 5:
                theta[1,k_n] = 5
            if theta[2,k_n] < 0.5:
                theta[2,k_n] = 0.5
            if theta[3,k_n] < 0.1:
                theta[3,k_n] = 0.1

        return theta

    def get_baseline(self, _span):
        if self.reward_history_.size < _span:
            b = np.mean(self.reward_history_)
        else:
            b = np.mean(self.reward_history_[-_span:])

        return b

    def calculate_reward(self, _state, _h_ceiling):         # _state is size 13 x timesteps
        state = _state
        h_ceiling = _h_ceiling
        h_delta = 0.02     # 0.044

        z = state[2,:]
        idx_zmax = np.where(z==z.max())[0][-1]

        state = state[:,0:idx_zmax]
        z = state[2,:]
        quat = state[4:7,:]
        quat = np.append(quat, state[3,:][np.newaxis,:], axis=0)          # rearrange quat as scalar-last format used in scipy Rotation

        loc = np.where( z>(h_ceiling-h_delta) )[0]
        if loc.size > 0:
            z[loc] = -z[loc]
        r1 = z / (h_ceiling-h_delta)

        r2 = np.zeros_like(r1)
        for k_quat in range(quat.shape[-1]):
            R = Rotation.from_quat(quat[:,k_quat])
            b3 = R.as_matrix()[:,2]                         # body z-axis

            r2[k_quat] = np.dot(b3, np.array([0,0,-1]))
        
        r = r1 + (r2+1)/2
        r_cum = np.zeros_like(r)

        temp = 0
        for k_r in range(0,len(r)):
            temp = r[k_r] + self.gamma_*temp
            r_cum[k_r] = temp

        if r_cum.size > 0:
            return r_cum[-1]
        else:
            return np.nan

class rlPEPGAgent_reactive:
    def __init__(self, _alpha_mu, _alpha_sigma, _gamma=0.95, _n_rollout = 6):
        self.alpha_mu_, self.alpha_sigma_,  = _alpha_mu, _alpha_sigma
        self.gamma_, self.n_rollout_ = _gamma, _n_rollout

        self.mu_ = np.array([[1.5], [-10.0]])           # mu    is size of  2 x 1
        self.sigma_ = np.array([[0.5], [0.5]])         # sigma is size of  2 x 1
        self.mu_history_ = copy.copy(self.mu_)
        self.sigma_history_ = copy.copy(self.sigma_)
        self.reward_history_ = np.array([0])

    def train(self, _theta, _reward):
        theta = _theta
        b = self.get_baseline(_span=3)
        #self.alpha_mu_ = self.alpha_mu_ * 0.9
        #self.alpha_sigma_ = self.alpha_sigma_ * 0.9

        T = theta - self.mu_
        S = (T*T - self.sigma_*self.sigma_)/self.sigma_
        r = _reward - b
        
        self.mu_ = self.mu_ + self.alpha_mu_*np.dot(T,r)
        self.sigma_ = self.sigma_ + self.alpha_sigma_*np.dot(S,r)

        for k in range(self.sigma_.size):
            if self.sigma_[k] <= 0:
                self.sigma_[k] = 0.001
        
        self.mu_history_ = np.append(self.mu_history_, self.mu_, axis=1)
        self.sigma_history_ = np.append(self.sigma_history_, self.sigma_, axis=1)
        self.reward_history_ = np.append(self.reward_history_, np.mean(_reward))

    def get_theta(self):
        theta = np.random.normal(self.mu_, self.sigma_, [self.mu_.size, self.n_rollout_])     # theta is size of  mu.size x n_runPerEp

        for k_n in range(self.n_rollout_):
            if theta[0,k_n] < 0:
                theta[0,k_n] = 0.001
            if theta[1,k_n] > 0:
                theta[1,k_n] = 0

        return theta

    def get_baseline(self, _span):
        if self.reward_history_.size < _span:
            b = np.mean(self.reward_history_)
        else:
            b = np.mean(self.reward_history_[-_span:])

        return b

    def calculate_reward(self, _state, _h_ceiling):         # _state is size 13 x timesteps
        state = _state
        h_ceiling = _h_ceiling
        h_delta = 0.02     # 0.044

        z = state[2,:]
        quat = state[4:7,:]
        quat = np.append(quat, state[3,:][np.newaxis,:], axis=0)          # rearrange quat as scalar-last format used in scipy Rotation

        r1 = z / h_ceiling

        r2 = np.zeros_like(r1)
        for k_quat in range(quat.shape[-1]):
            R = Rotation.from_quat(quat[:,k_quat])
            b3 = R.as_matrix()[:,2]                         # body z-axis

            r2[k_quat] = np.dot(b3, np.array([0,0,-1]))
            if (r2[k_quat]>0.8) and (z[k_quat] > 0.8*h_ceiling):            # further incentivize when b3 is very close to -z axis
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

class rlsysPEPGAgent_reactive:
    def __init__(self, _alpha_mu, _alpha_sigma, _gamma=0.95, _n_rollout = 6):
        self.alpha_mu_, self.alpha_sigma_,  = _alpha_mu, _alpha_sigma
        self.gamma_, self.n_rollout_ = _gamma, _n_rollout

        self.mu_ = np.array([[5.0], [-10.0]])   # Initial estimates of mu: size (2 x 1)
        self.sigma_ = np.array([[1], [1]])      # Initial estimates of sigma: size (2 x 1)
        self.mu_history_ = copy.copy(self.mu_)  # Creates another array of self.mu_ and attaches it to self.mu_history_
        self.sigma_history_ = copy.copy(self.sigma_)
        self.reward_history_ = np.array([0])

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

    def get_theta(self):
        zeros = np.zeros_like(self.mu_)
        epsilon = np.random.normal(zeros, self.sigma_, [zeros.size, self.n_rollout_])     # theta is size of  mu.size x n_runPerEp

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

    def calculate_reward(self, _state, _h_ceiling):         # _state is size 13 x timesteps
        state = _state
        h_ceiling = _h_ceiling
        h_delta = 0.02     # 0.044

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
        
        #plt.figure()
        #plt.plot(t,y[0,:])
        #plt.show()

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
        file_name2 = '/home/pan/catkin_ws2/src/4. rl/src/' + file_name + '.mat'
        
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
