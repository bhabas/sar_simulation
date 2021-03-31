import numpy as np
import copy
from rl_syspepg import ES
from math import floor,log,sqrt,ceil
import time

class CMA(ES):
    # adapted from https://arxiv.org/pdf/1604.00772.pdf
    def __init__(self,n):

        self.alpha_mu = np.array([[0.0],[0.0] ])#[2.0]] )#,[0.1]])
        self.alpha_sigma = np.array([[0.0],[0.0] ])#, [1.0]])#,[0.05]])
        self.N = n # dimensions to learn

        # initialize mean and learning rate
        self.mu = np.random.rand(self.N,1)
        self.mu = np.array([[8.0],[8.0]])
        self.sigma_lr = 0.5
        self.counteval = 0


        self.lamda =  4.0 +  4.0 + floor(3.0*log(self.N))  # number of population members
        self.n_rollout = int(self.lamda/2.0)  # same as lamda
        self.mu_s = floor(self.lamda/4) # number of sample members from each populatoin //added floor /2
        
        # determine weights for top mu members of population for mean+sig update
        self.weights = log(self.mu_s + 0.5) - np.transpose(np.log(np.linspace(1,self.mu_s,self.mu_s))) # transpose???
        self.weights = self.weights/np.sum(self.weights) # normalize

        # calculate effective members of population given weights
        self.mueff= ((np.sum(self.weights))**2)/np.sum(np.square(self.weights))

        print(self.N,self.lamda,self.mu_s,self.mueff)
        print(self.weights)

        # try different learning parameters from text
        self.cc = (4.0 + self.mueff/self.N)/(self.N + 4.0 + 2.0*self.mueff/self.N)  # time horizon for C (how far back lineages influnce) 
        self.cs = (self.mueff + 2.0)/(self.N + self.mueff + 5.0)  # t-const for cumulation forsigma control   
        self.c1 = 2.0/((self.N+1.3)**2 + self.mueff)  # maybe wrong!!! rank one update learning rate
        self.cmu = 2.0*(self.mueff - 2.0 + 1.0/self.mueff)/((self.N + 2.0)**2 + 2.0*self.mueff/2.0)  # rank mu update learning rate 
        self.damps = 1.0 + 2.0*max(0, sqrt((self.mueff - 1.0)/(self.N + 1.0)) - 1.0) + self.cs # damping term for sigma learning rate
  
        self.pc = np.zeros((self.N,1))
        self.ps = np.zeros((self.N,1))   # evolution paths for C and sigma   (basically how they mueffd)
        #print(self.ps)
        self.B = np.identity(self.N)    # initizlize eigenvector
        self.D = np.identity(self.N)   # initizlize eigenvalue  
        self.C = (self.B.dot(self.D)).dot(np.transpose(self.B.dot(self.D)))   # construct covariance matrix from eigenvec/val  
        
        self.sigma = np.array([self.C[0,0],self.C[1,1],self.C[1,0] ])
        # try defining B and D from C instead
        self.C = 2*np.identity(self.N)
        self.D,self.B = np.linalg.eig(self.C)    
        self.D = np.diag(self.D)

        
        self.eigeneval = 0      # counter  
        self.chiN = (self.N**0.5)*(1.0 - 1.0/(4.0*self.N) + 1.0/(21.0*(self.N**2)))  # expectation estimation for normal dist?

    def get_theta(self):
        # construct new population of lamda children  
        lamda = int(self.lamda)
        self.arz = np.zeros((self.N,lamda)) 
        self.arx = np.zeros((self.N,lamda)) 
        for k in range(0,lamda):
            mm = np.random.randn(self.N,1) # standard normally distributed vector 
            #print(mm) 
            self.arz[0,k] = mm[0]   # base mean for rrev
            self.arz[1,k] = mm [1]  # base mean for gain rate
            #print(self.arz)

            #arx = self.mu + self.sigma_lr*(np.dot(self.B*self.D,mm) )  # add mutation    % Eq.40       
            arx = self.mu + self.sigma_lr*(self.B.dot(self.D)).dot(mm) # mutate child using covariance matrix and base mean
            #try adding mu - sigma for symmetric sampling
            
            #print(arx)
            #print(arx2)
            self.arx[0,k] = arx[0]*(arx[0] > 0)  # enforce stricly postive values (gain rate is negated in start_training.py)
            self.arx[1,k] = arx[1]*(arx[0] > 0)
            self.counteval +=2 # increase species number counter

        theta = copy.copy(self.arx)
        return theta,0

    def train(self,theta,reward, epsilon=0):
        # combine theta and reward and sort by highest reward
        summary = np.concatenate((np.transpose(theta),reward,np.transpose(self.arz)),axis=1)
        summary = np.transpose(summary[summary[:,2].argsort()[::-1]])

        # weighted recombination of mmu best members
        mu = np.dot(summary[0:2,0:self.mu_s],np.transpose(self.weights))
        zmean = np.dot(summary[3:5,0:self.mu_s],np.transpose(self.weights))
        print(summary)
        print(mu)
        print(zmean)
        # reshape to column vector
        self.mu = mu.reshape(2,1)
        self.zmean = zmean.reshape(2,1)


        # calculate current evolution path
        self.ps = (1-self.cs)*self.ps + (sqrt(self.cs*(2-self.cs)*self.mueff))*self.B.dot(self.zmean)   # Eq.43
        #print(self.ps)
        #print('Evolution path')
        #print(self.ps)    
        self.hsig = np.linalg.norm(self.ps)/sqrt(1-(1-self.cs)**(2*self.counteval/self.lamda))/self.chiN < 1.4+2/(self.N+1)
        #print(self.hsig)    

        self.pc = (1-self.cc)*self.pc + self.hsig*sqrt(self.cc*(2-self.cc)*self.mueff)*(self.B.dot(self.D)).dot(self.zmean)

        #print(self.pc)
        # Update covariance matrix (might have errors) -> need to understand this better
        C = (1-self.c1 -self.cmu)*self.C  # contribution of previous  C
        #print(C)
        C = C + self.c1*(self.pc*np.transpose(self.pc)+ (1-self.hsig)*self.cc*(2.0 - self.cc)*self.C)
        #print(C)
        temp = (self.B.dot(self.D)).dot(summary[3:5,0:self.mu_s])
        #print(temp)
        C = C + self.cmu*( (temp.dot(np.diag(self.weights))).dot(np.transpose(temp))  )
        #print(self.cmu*( (temp.dot(np.diag(self.weights))).dot(np.transpose(temp))  ))
        #print(self.cmu)
        #print(C)
        self.C = C
        # adjust lerning rate
        self.sigma_lr = self.sigma_lr*np.exp((self.cs/self.damps)*(np.linalg.norm(self.ps)/self.chiN - 1.0))
        #print(self.sigma_lr)

        # force covariance matrix to be symmetric
        self.eigeneval = self.counteval      
        self.C = np.triu(self.C)+ np.transpose(np.triu(self.C,1))       
        self.D,self.B = np.linalg.eig(self.C)    
        # if theres an error, its probably here!!!!!!! 
        print(self.D)

        self.sigma = np.array([self.C[0,0],self.C[1,1],self.C[1,0] ])
        self.D = np.sqrt(np.diag(self.D))
        print(self.D)
        print(np.sqrt(np.diag(self.D)))
        #self.D = np.sqrt(np.diag(self.D))
        #self.D = np.diag(np.sqrt(np.diag(self.D))) # <------- !!!!
        print(self.D)
        print(self.C)
        #print(C)print
        #print(self.D)
        #print(self.B)
        #print(self.C)

class CMA_sym(CMA):
    def get_theta(self): # try 16 pop / sym might make it okay
        # construct new population of lamda children  
        lamda = int(self.lamda)
        self.arz = np.zeros((self.N,lamda)) 
        self.arx = np.zeros((self.N,lamda)) 
        for k in range(0,int(lamda/2)): # need lamda to be even so 20/4 = 5 bad 24/4 = 6 good 
            mm = np.random.randn(self.N,1) # standard normally distributed vector 
            #print(mm) 
            self.arz[0,k] = mm[0]   # base mean for rrev
            self.arz[1,k] = mm [1]  # base mean for gain rate
            #print(self.arz)

            #arx = self.mu + self.sigma_lr*(np.dot(self.B*self.D,mm) )  # add mutation    % Eq.40       
            arx = self.mu + self.sigma_lr*(self.B.dot(self.D)).dot(mm) # mutate child using covariance matrix and base mean
            arx2 = self.mu - self.sigma_lr*(self.B.dot(self.D)).dot(mm) # symmetric sample
            #try adding mu - sigma for symmetric sampling
            print(arx,arx2)
            #print(arx)
            #print(arx2)
            self.arx[0,2*k] = arx[0]*(arx[0] > 0)  # enforce stricly postive values (gain rate is negated in start_training.py)
            self.arx[1,2*k] = arx[1]*(arx[0] > 0)

            self.arx[0,2*k +1] = arx2[0]*(arx2[0] > 0)  # enforce stricly postive values (gain rate is negated in start_training.py)
            self.arx[1,2*k + 1] = arx2[1]*(arx2[0] > 0)
            self.counteval +=1 # increase species number counter

        theta = copy.copy(self.arx)
        return theta,0

if __name__ == "__main__":
    es = CMA(2)
    #print(es.N)
    #print(es.mu)
    theta = es.get_theta()
    #print(theta)
    reward = np.array([[10],[5],[3],[2],[12],[20]])
    es.train(theta,reward)

class CMA_basic(ES):
    # expand on update
    # https://arxiv.org/pdf/1604.00772.pdf
    def __init__(self,mu ,sigma,N_best = 0.25,n_rollout = 10):
        # not needed alpha (but doenst work without)
        self.alpha_mu = np.array([[0.0],[0.0] ])#[2.0]] )#,[0.1]])
        self.alpha_sigma = np.array([[0.0],[0.0] ])#, [1.0]])#,[0.05]])
        self.mu = mu
        self.sigma = sigma
        self.N_best = N_best
        self.n_rollout = int(n_rollout/2)  # to use with pepg framework

        self.mu_history = copy.copy(self.mu)  # Creates another array of self.mu_s and attaches it to self.mu_s_history
        self.sigma_history = copy.copy(self.sigma)
        self.reward_history = np.array([0])

    def get_theta(self):
        # mueff format for np multivariate normal function
        mu = np.array([self.mu[0,0],self.mu[1,0]])

        sigma = np.array([[self.sigma[0,0],self.sigma[2,0]],[self.sigma[2,0],self.sigma[1,0]]])
        theta = np.random.multivariate_normal(mu,sigma,[1, int(self.n_rollout*2)])
        theta = theta[0,:,:]
        theta = np.transpose(theta)

        
        ## Caps values of RREV to be greater than zero and Gain to be neg so
        # the system doesnt fight itself learning which direction to rotate
        for k_n in range(int(self.n_rollout*2)):
            if theta[0,k_n] < 0:
                theta[0,k_n] = 0.001
            if theta[1,k_n] > 0:
                theta[1,k_n] = -0.001

        return theta,0

    def train(self,theta,reward,epsilon=0):
        summary = np.append(np.transpose(theta),reward,axis=1)
        summary = summary[summary[:,2].argsort()[::-1]]
        N = ceil(self.N_best*self.n_rollout*2.0)

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