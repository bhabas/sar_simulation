## STANDARD IMPORTS
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

## PYTORCH IMPROTS
import torch
import torch.nn as nn
import torch.nn.functional as F

## SKLEARN IMPORTS
from sklearn.model_selection import train_test_split
from sklearn.metrics import *
from sklearn import preprocessing
from sklearn.svm import OneClassSVM


import plotly.graph_objects as go

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping"



if __name__ == "__main__":

    ## SET SEEDS
    torch.manual_seed(0)
    np.random.seed(0)

    model_initials = "NL_DR"

    ## LOAD DATA
    df_raw = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_DR/NL_LR_Trials_DR.csv").dropna() # Collected data
    df_raw = df_raw.query("phi_IC >= 86")

    ## ORGANIZE DATA
    Tau = df_raw["Tau_flip_mean"]
    OF_y = df_raw["OFy_flip_mean"]
    d_ceil = df_raw["D_ceil_flip_mean"]
    landing_rate = df_raw["LR_4leg"]

    X = np.stack((Tau,d_ceil),axis=1)
    y = landing_rate

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING SETS
    data_array = np.stack((Tau,d_ceil,landing_rate),axis=1)
    df = pd.DataFrame(data_array,columns=['Tau','d_ceil','LR'])
    df = df.sort_values(by='LR')

    train_df, test_df = train_test_split(df,test_size=0.10,random_state=73)
    X_train = train_df[['Tau','d_ceil']].to_numpy()
    y_train = train_df[['LR']].to_numpy()

    X_test = test_df[['Tau','d_ceil']].to_numpy()
    y_test = test_df[['LR']].to_numpy()


    Param_Path = f'{BASEPATH}/NeuralNetwork/Info/NN_Layers_{model_initials}.h'

    clf = OneClassSVM(nu=0.3,gamma=80,kernel='rbf')
    clf.fit(X_train)

    xx2, yy2 = np.meshgrid(np.linspace(0.1,0.4, 500), np.linspace(0,1.2, 500))
    Z2 = clf.decision_function(np.c_[xx2.ravel(), yy2.ravel()])
    Z2 = Z2.reshape(xx2.shape)



    fig = plt.figure()
    ax = fig.add_subplot(111)

    ax.contour(xx2, yy2, Z2, levels=[0], linewidths=2, colors='k')
    ax.scatter(X_train[:,0],X_train[:,1],color='tab:blue')
    ax.grid()
    ax.set_xlim(0.1,0.4)
    ax.set_ylim(0,1.2)
    ax.set_xlabel('Tau')
    ax.set_ylabel('D_ceil')

    plt.show()