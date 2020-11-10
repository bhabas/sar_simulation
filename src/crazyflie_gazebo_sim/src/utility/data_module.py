import pandas as pd
import numpy as np


def run_data(df,k_ep,k_run): # return dataframe for run along with the IC
    run_df = df.loc[ (df['k_ep'] == k_ep) & (df['k_run'] == k_run)]
    IC_df = run_df.tail(1) # Grab last row from run_df
    run_df = run_df.iloc[:-1] # Drop last row from run_df
    return run_df,IC_df

def grab_data(run_df,clmn_name): # return array of vals from dataframe
    val_array = run_df[clmn_name].to_numpy()
    return val_array

def grab_all_IC(df): # return df of just IC
    df = df[pd.notnull(df['policy'])] # only grab rows not missing policy data
    return df

def grab_IC(IC_df,clmn_name): # returns array of IC for both single run and all runs
    arr = []
    for val in IC_df[clmn_name]:
        if type(val) == str:
            val = np.fromstring(val[2:-2], dtype=float, sep=' ')
        arr.append(val)
    arr = np.asarray(arr)
    return arr 
        