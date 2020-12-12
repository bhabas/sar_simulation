import numpy as np
import pandas as pd



class DataFile:
    def __init__(self,filepath):
        self.trial_df = pd.read_csv(filepath)

    def grab_run(self,k_ep,k_run):
        run_df = self.trial_df[(self.trial_df['k_ep']==k_ep) & (self.trial_df['k_run']==k_run)]
        return run_df



    def grab_policy(self,k_ep,k_run):
        pass

    

    def plot_rewardFunc(self):
        pass

    def plot_state(self,k_ep,k_run,state):
        pass

    ## Grab final policy
    ## Plot reward function
    ## Plot State across episode
