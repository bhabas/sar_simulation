import numpy as np
import copy
from rl_syspepg import ES
from math import floor,log,sqrt
class CMA(ES):
    def __init__(self,n):
        self.N = n # dimension
        self.mean = np.random.rand(self.N,1)
        self.sigma = 0.5

        self.lamda = 4.0 + floor(3.0*log(self.N))
        self.mu = floor(self.lambda/2) # added floor
        self.weights = log(self.mu + 0.5) - np.transpose(np.log(np.linspace(1,self.mu,self.mu))) # transpose???
        self.weights = self.weights/np.sum(self.weights)
        self.mueff= (np.sum(np.square(self.weights)))/np.sum(np.square(self.weights))

        self.cc = (4.0 + self.mueff/self.N)/(self.N + 4.0 + 2.0*self.mueff/self.N)  # time constant for cumulation for C   
        self.cs = (self.mueff + 2.0)/(self.N + self.mueff + 5.0)  # t-const for cumulation forsigma control   
        self.c1 = 2.0/((self.N+1.3)**2 + self.mueff)  # maybe wrong!!! learning rate for rank-one update of C
        self.cmu = 2.0*(self.mueff - 2.0 + 1.0/self.mueff)/((self.N + 2.0)**2 + 2.0*self.mueff/2.0)  # and for rank-mu update
        self.damps = 1.0 + 2.0*max(0, sqrt((self.mueff - 1.0)/(self.N + 1.0)) - 1.0) + self.cs # damping for sigm

        # Initialize dynamic (internal) strategy parameters andconstants   
        self.pc = np.zeros((self.N,1))
        self.ps = np.zeros((self.N,1))   # evolution paths for C and sigma   
        self.B = np.identity(self.N)    # B defines the coordinate system   
        self.D = np.identity(self.N)   #diagonal matrix D defines the scaling   
        self.C = self.B*self.D*np.transpose((self.B*self.D))   # covariance matrix   
        self.eigeneval = 0      #B and D updated at counteval == 0   
        self.chiN = (self.N**0.5)*(1.0 - 1.0/(4.0*self.N) + 1.0/(21.0*(self.N**2)))  # expectation of43%   ||N(0,I)|| == norm(randn(N,1))

    def get_theta(self):
        # Generate and evaluate lambda offspring     
        for k in range(1:lambda):
            self.arz(:,k) = np.random.randn(N,1) # standard normally distributed vector       
            self.arz(arx(:,k) = xmean + sigma*(B*D*arz(:,k));   # add mutation    % Eq.40       
            arfitness(k) = feval(strfitnessfct, arx(:,k)); # objective function call       
            counteval = counteval+1;

    def train(self,theta,reward):
        pass

if __name__ == "__main__":
    es = CMA(2)
    print(es.N)
    print(es.mean)


class CMA_basic(ES):
    # expand on update
    # https://arxiv.org/pdf/1604.00772.pdf
    def __init__(self,mu ,sigma,N_best = 0.25,n_rollout = 10,gamma=0.95):
        self.gamma = gamma
        self.mu = mu
        self.sigma = sigma
        self.N_best = N_best
        self.n_rollout = n_rollout

        self.mu_history = copy.copy(self.mu)  # Creates another array of self.mu and attaches it to self.mu_history
        self.sigma_history = copy.copy(self.sigma)
        self.reward_history = np.array([0])

    def get_theta(self):
        # change format for np multivariate normal function
        mu = np.array([self.mu[0,0],self.mu[1,0]])

        sigma = np.array([[self.sigma[0,0],self.sigma[2,0]],[self.sigma[2,0],self.sigma[1,0]]])
        theta = np.random.multivariate_normal(mu,sigma,[1, self.n_rollout])
        theta = theta[0,:,:]
        theta = np.transpose(theta)

        ## Caps values of RREV to be greater than zero and Gain to be neg so
        # the system doesnt fight itself learning which direction to rotate
        for k_n in range(self.n_rollout):
            if theta[0,k_n] < 0:
                theta[0,k_n] = 0.001
            if theta[1,k_n] > 0:
                theta[1,k_n] = -0.001

        return theta

    def train(self,theta,reward):
        summary = np.append(np.transpose(theta),reward,axis=1)
        summary = summary[summary[:,2].argsort()[::-1]]
        N = ceil(self.N_best*self.n_rollout)

        x = summary[0:N,0]
        y = summary[0:N,1]

        ux = np.sum(x)/N
        uy = np.sum(y)/N

        mux = self.mu[0,0]
        muy = self.mu[1,0]

        sx = np.sum(np.square(x - mux))/N
        sy = np.sum(np.square(y - muy))/N

        sxy = np.sum(np.dot(x - mux,y - muy))/N
        self.mu = np.array( [[ux],[uy]])
        self.sigma = np.array( [[sx],[sy],[sxy]])

        for k in range(self.sigma.size-1): #  If sigma oversteps negative then assume convergence
            if self.sigma[k] <= 0:
                self.sigma[k] = 0.05
        
        if self.sigma[-1] == 1:
            self.sigma = 0.9
        elif self.sigma[-1] == -1:
            self.sigma = -0.9
        
        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)
        self.reward_history = np.append(self.reward_history, np.mean(reward))