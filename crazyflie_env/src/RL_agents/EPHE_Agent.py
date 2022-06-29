import numpy as np
import scipy.stats 

class EPHE_Agent():
    ## EM‑based policy hyper parameter exploration
    ## CITED HERE: DOI 10.1007/s10015-015-0260-7
    def __init__(self,mu=[0,0,0],sigma=[0,0,0], n_rollouts = 6):
        
        self.agent_type = 'EPHE'
        self.n_rollouts = n_rollouts
        self.mu = np.array(mu).reshape(-1,1)
        self.sigma = np.array(sigma).reshape(-1,1)

        self.n_rollouts = n_rollouts
        self.dim = len(self.mu)


    def get_theta(self):

        theta = np.zeros((len(self.mu),self.n_rollouts))
        
        ## SET UPPER AND LOWER LIMITS FOR TAU AND My
        lower_limit = [0.0,0.0]     # Lower and Upper limits for truncated normal distribution 
        upper_limit = [6.0,10.0]    # 9.10 N*mm is the upper limit for My_d
        eps = 1e-6 # Prevent divide by zero error
                                
        ## SAMPLE VALUES FROM TRUNCATED NORMAL DISTRIBUTION
        for ii,mu_ii in enumerate(self.mu):
            theta[ii,:] = scipy.stats.truncnorm.rvs(
                (lower_limit[ii]-mu_ii)/(self.sigma[ii]+eps),
                (upper_limit[ii]-mu_ii)/(self.sigma[ii]+eps),
                loc=mu_ii,
                scale=self.sigma[ii],
                size=self.n_rollouts
            )      

        return theta

    def train(self,theta,reward,k=3):
        
        ## SORT THETA/REWARD ARRAYS (TODO: CLEAN LOGIC)
        reward = reward.reshape(-1,1)
        summary = np.concatenate((np.transpose(theta),reward),axis=1)
        summary = np.transpose(summary[summary[:,self.dim].argsort()[::-1]])
        # print(summary)
        
        theta_h = summary[0:self.dim,0:k]
        R_h = summary[self.dim,0:k].reshape(-1,1)


        ## CALC SUMMATIONS
        S_theta = (theta_h.dot(R_h))
        S_reward = np.sum(R_h)
        S_diff = np.square(theta_h - self.mu).dot(R_h)

        ## CALC UPDATED MU AND SIGMA VECTORS
        self.mu = S_theta/(S_reward + 1e-3)
        self.sigma = np.sqrt(S_diff/(S_reward + 1e-3))
    

if __name__ == "__main__":
    np.set_printoptions(precision=2, suppress=True)
    mu = np.array([[1.0],[4.0]])
    sigma = np.array([[0.1],[1.5]])

    agent = EPHE_Agent(mu,sigma)

    theta = agent.get_theta()
    reward = np.array([[11, 20, 11, 40, 32, 45, 80, 80]])

    agent.train(theta,reward)
