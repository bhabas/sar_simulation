import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



class DataFile:
    def __init__(self,filepath):
        self.trial_df = pd.read_csv(filepath)

    def select_run(self,k_ep,k_run): ## Create run dataframe from k_ep and k_run
        run_df = self.trial_df[(self.trial_df['k_ep']==k_ep) & (self.trial_df['k_run']==k_run)]
        return run_df

    def grab_IC(self):

        """Return IC values

        Returns:
            float: Initial Velocity Conditions (vx,vy,vz)
        """        
        vx_IC = self.trial_df.iloc[-1]['vx']
        vy_IC = self.trial_df.iloc[-1]['vy']
        vz_IC = self.trial_df.iloc[-1]['vz']
        
        return vx_IC,vy_IC,vz_IC

    def grab_finalPolicy(self):
        """Returns the final policy for a trial

        Returns:
           mu [np.array]: Final policy average
           sigma [np.array]: Final policy standard deviation

        """        
        
        ## FIND FINAL RUN DF
        k_ep = self.trial_df.iloc[-1]['k_ep']
        k_run = self.trial_df.iloc[-1]['k_run']
        run_df = self.select_run(k_ep,k_run)

        ## SELECT MU & SIGMA
        mu = run_df.iloc[-1]['mu']
        mu = np.fromstring(mu[2:-2], dtype=float, sep=' ')  # Convert str to np array
        mu = np.expand_dims(mu, axis=0)                     # Expand array from (n,) to (n,1)

        sigma = run_df.iloc[-1]['sigma']
        sigma = np.fromstring(sigma[2:-2], dtype=float, sep=' ')
        sigma = np.expand_dims(sigma, axis=0)

        return mu,sigma

    def grab_stateData(self,k_ep,k_run,stateName):
        """Returns np.array of specified state

        Args:
            k_ep (int): Episode number
            k_run (int): Run number
            stateName (int): State name

        Returns:
            state [np.array]: Returned state values
        """        
        run_df = self.select_run(k_ep,k_run)
        run_df = run_df.iloc[:-1] # Drop last row of DF

        ## GRAB STATE DATA AND CONVERT TO NUMPY ARRAY
        state = run_df.iloc[:][stateName]
        state = state.to_numpy()
        state = np.expand_dims(state, axis=0).T

        return state
    
    def plot_rewardFunc(self,figNum=0):
        """Plot rewards for trial

        Args:
            figNum (int, optional): Figure Number. Defaults to 0.
        """        
        num_rollouts = self.trial_df.iloc[-1]['n_rollouts']

        ## CREATE ARRAYS FOR REWARD, K_EP
        reward_df = self.trial_df.iloc[:][['k_ep','reward']].dropna()
        rewards_arr = reward_df.to_numpy()
        rewards = rewards_arr[:,1]
        k_ep_r = rewards_arr[:,0]

        ## CREATE ARRAYS FOR REWARD_AVG, K_EP
        rewardAvg_df = reward_df.groupby('k_ep').mean()
        rewards_avg = rewardAvg_df.to_numpy()
        k_ep_ravg = reward_df.k_ep.unique()


        fig = plt.figure(figNum)
        ax = fig.add_subplot(111)
        ax.scatter(k_ep_r,rewards,marker='_',color='black',alpha=0.5,label='Reward')
        ax.scatter(k_ep_ravg,rewards_avg,marker='o',color='red',label='Reward_avg')
        


        ax.set_ylabel("Rewards")
        ax.set_xlabel("k_ep")
        ax.set_xlim(-2,20)
        ax.set_ylim(-2,25)
        ax.set_title(f"Reward vs Episode | Rollouts per Episode: {num_rollouts*2}")
        ax.legend()
        ax.grid()

        plt.show()

    def plot_state(self,k_ep,k_run,stateName,figNum=0):
        """Plot state data from run vs time

        Args:
            k_ep (int): Ep. Num
            k_run (int): Run. Num
            stateName (str): State name from .csv
            figNum (int, optional): Figure Number. Defaults to 0.
        """        

        ## GRAB RUN DF
        run_df = self.select_run(k_ep,k_run)
        run_df = run_df.iloc[:-1] # Drop last row of DF

        ## GRAB STATE/TIME DATA AND CONVERT TO NUMPY ARRAY
        state = run_df.iloc[:][stateName]
        state = state.to_numpy()
        state = np.expand_dims(state, axis=0).T

        time = run_df.iloc[:]['t']
        time = time.to_numpy()
        time = np.expand_dims(time, axis=0).T
        time = time - np.min(time)

        
        ## PLOT STATE/TIME DATA
        fig = plt.figure(figNum)
        ax = fig.add_subplot(111)
        ax.plot(time,state,label=f"{stateName}")


        ax.set_ylabel(f"{stateName}")
        ax.set_xlabel("time [s]")
        ax.legend()
        ax.grid()

        plt.show()

    def plot_policy(self,trialNum=np.nan):
        """Creates subplots to show convergence for policy gains

        Args:
            trialNum ([int], optional): Display trial number. Defaults to np.nan.
        """        

        ## CLEAN AND GRAB DATA FOR MU & SIGMA
        policy_df = self.trial_df.iloc[:][['k_ep','mu','sigma']]
        policy_df = policy_df.dropna().drop_duplicates()
        k_ep_arr = policy_df.iloc[:]['k_ep'].to_numpy()

        

        ## CREATE NP ARRAYS FOR MU & SIGMA OVER TRIAL
        mu_arr = []
        for mu in policy_df.iloc[:]['mu']:
            mu = np.fromstring(mu[2:-2], dtype=float, sep=' ')
            mu_arr.append(mu)
        mu_arr = np.array(mu_arr)

        sigma_arr = []
        for sigma in policy_df.iloc[:]['sigma']:
            sigma = np.fromstring(sigma[2:-2], dtype=float, sep=' ')
            sigma_arr.append(sigma)
        sigma_arr = np.array(sigma_arr)

        

        num_col = mu_arr.shape[1] # Number of policy gains in mu [Currently 3]
        G_Labels = ['RREV_trigger','G1','G2','G3','G4','G5'] # List of policy gain names
        vx,vy,vz = self.grab_IC()


        ## CREATE SUBPLOT FOR MU 
        fig = plt.figure()
        ax = fig.add_subplot(211)
        for jj in range(num_col): # Iterate through gains and plot each
            ax.plot(k_ep_arr,mu_arr[:,jj],label=G_Labels[jj])


        ax.set_ylabel('Policy Values')
        ax.set_xlabel('K_ep')
        ax.set_ylim(0) # Set lower ylim
        ax.set_title(f'Policy Value vs Episode (Trial:{trialNum}) | Vx = {vx} Vz = {vz}')
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.25),ncol=num_col)
        ax.grid()


        ## CREATE SUBPLOT FOR SIGMA
        ax = fig.add_subplot(212)
        for jj in range(num_col): # Iterate through gains and plot each
            ax.plot(k_ep_arr,sigma_arr[:,jj],label=G_Labels[jj])

        ax.set_ylabel('Standard Deviation')
        ax.set_xlabel('K_ep')
        ax.set_title(f'Policy S.D. vs Episode (Trial:{trialNum}) | Vx = {vx} Vz = {vz}')
        ax.legend(ncol=3,loc='upper right')
        ax.grid()

        fig.tight_layout()
        plt.show()
