import numpy as np
import pandas as pd



class DataFile:
    def __init__(self,filepath):
        self.trial_df = pd.read_csv(filepath)

    def select_run(self,k_ep,k_run): ## Creat run dataframe from k_ep and k_run
        run_df = self.trial_df[(self.trial_df['k_ep']==k_ep) & (self.trial_df['k_run']==k_run)]
        return run_df



    def grab_finalPolicy(self):
        """
        Returns the final policy for a trial

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
    

    def plot_rewardFunc(self):
        pass

    def plot_state(self,k_ep,k_run,state):
        pass

    ## Grab final policy
    ## Plot reward function
    ## Plot State across episode
