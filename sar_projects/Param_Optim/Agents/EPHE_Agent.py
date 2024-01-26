import numpy as np
import scipy.stats 
import time
import rospy
from sar_msgs.msg import RL_Data,RL_History

class EPHE_Agent():
    ## EM‑based policy hyper parameter exploration
    ## CITED HERE: DOI 10.1007/s10015-015-0260-7
    def __init__(self,mu=[0,0],sigma=[0,0], n_rollouts = 6):
        
        self.agent_type = 'EPHE'
        self.n_rollouts = n_rollouts
        self.mu = np.array(mu).reshape(-1,1)
        self.sigma = np.array(sigma).reshape(-1,1)

        self.dim = len(self.mu)

        self.RL_Data_Publisher = rospy.Publisher('/RL/Data',RL_Data,queue_size=10)
        self.RL_Convg_Publisher = rospy.Publisher('/RL/History',RL_History,queue_size=10)

        ## CONVERGENCE HISTORY
        self.K_ep_list = []
        self.K_run_list = []

        self.mu_1_list = []         # List of mu values over course of convergence
        self.mu_2_list = [] 

        self.sigma_1_list = []      # List of sigma values over course of convergence
        self.sigma_2_list = []

        self.reward_list = []       # List of reward values over course of convergence
        self.reward_avg_list = []   # List of reward averages ""
        self.Kep_list_reward_avg = []

        ## PARAM OPTIM DATA
        self.K_ep = 0                   # Episode number
        self.K_run = 0                  # Run number
        self.error_str = ""

        self.V_mag_rel = 0.0            # Relative velocity magnitude [m/s]
        self.V_angle_rel = 0.0          # Relative velocity angle [deg]
        self.Plane_Angle = 0.0          # Plane angle [deg]
        self.policy = [0.0,0.0]         # Policy sampled from Gaussian distribution

        self.reward = 0.0               # Calculated reward from run
        self.reward_avg = 0.0           # Averaged rewards over episode
        self.reward_vals = np.zeros(6)


        self.trialComplete_flag = False



    def RL_Publish(self):

        ## RL DATA
        RL_msg = RL_Data() ## Initialize RL_Data message
        
        RL_msg.n_rollouts = self.n_rollouts

        RL_msg.K_ep = self.K_ep
        RL_msg.K_run = self.K_run
        RL_msg.error_string = self.error_str
        RL_msg.n_rollouts = self.n_rollouts

        RL_msg.mu = list(self.mu.flatten())
        RL_msg.sigma = list(self.sigma.flatten())
        RL_msg.policy = self.policy

        RL_msg.reward = self.reward
        RL_msg.reward_avg = self.reward_avg
        RL_msg.reward_vals = self.reward_vals

        RL_msg.V_mag_rel = self.V_mag_rel
        RL_msg.V_angle_rel = self.V_angle_rel

        RL_msg.trialComplete_flag = self.trialComplete_flag
        self.RL_Data_Publisher.publish(RL_msg) ## Publish RL_Data message
        
        ## CONVERGENCE HISTORY
        RL_convg_msg = RL_History()
        RL_convg_msg.K_ep_list = self.K_ep_list
        RL_convg_msg.K_run_list = self.K_run_list


        RL_convg_msg.mu_1_list = self.mu_1_list
        RL_convg_msg.mu_2_list = self.mu_2_list

        RL_convg_msg.sigma_1_list = self.sigma_1_list
        RL_convg_msg.sigma_2_list = self.sigma_2_list

        RL_convg_msg.reward_list = self.reward_list

        RL_convg_msg.Kep_list_reward_avg = self.Kep_list_reward_avg
        RL_convg_msg.reward_avg_list = self.reward_avg_list

        self.RL_Convg_Publisher.publish(RL_convg_msg) ## Publish RL_Data message

        time.sleep(0.1)


    def get_theta(self):

        theta = np.zeros((len(self.mu),self.n_rollouts))
        
        ## SET UPPER AND LOWER LIMITS FOR TAU AND My
        lower_limit = [-1.0, -1.0] # Lower and Upper limits for truncated normal distribution 
        upper_limit = [1.0, 1.0]    
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
