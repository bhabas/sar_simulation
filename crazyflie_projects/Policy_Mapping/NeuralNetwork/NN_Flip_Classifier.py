## STANDARD IMPORTS
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from sklearn import datasets

## PYTORCH IMPROTS
import torch
import torch.nn as nn
import torch.nn.functional as F

## SKLEARN IMPORTS
from sklearn.model_selection import train_test_split
from sklearn.datasets import make_blobs,make_moons
from sklearn.metrics import *
from sklearn import preprocessing

import plotly.graph_objects as go


np.set_printoptions(suppress=True)
BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"

## DEFINE NN MODEL
class NN_Model(nn.Module):
    def __init__(self,in_features=3,h=10,out_features=1):
        super().__init__()
        self.fc1 = nn.Linear(in_features,h)     # Layer 1 
        self.fc2 = nn.Linear(h,h)               # Layer 2
        self.out = nn.Linear(h,out_features)    # Layer 3

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.elu(self.fc1(x))
        x = F.elu(self.fc2(x))
        x = torch.sigmoid(self.out(x))

        return x

class NN_Flip_Classifier():
    def __init__(self,model_config):
        self.model_config = model_config
        self.modelInitials()
        self.scaler = None
        self.loadScaler()

    def modelInitials(self): # RETURNS INITIALS FOR MODEL
        str = self.model_config
        charA = str[0] # [W]ide
        charB = str[str.find("-")+1] # [L]ong

        self.model_initials = charA+charB

    def createScaler(self,X):

        ## GENERATE SCALER
        self.scaler = preprocessing.StandardScaler().fit(X)
        np.stack((self.scaler.mean_,self.scaler.scale_),axis=1)

        ## SAVE SCALER TO FILE
        np.savetxt(f"{BASEPATH}/Info/Scaler_Flip_{self.model_initials}.csv",
            np.stack((self.scaler.mean_,self.scaler.scale_),axis=1),
            fmt='%.6f',
            delimiter=',',
            comments='',
            header=f'mean,std',
            footer='')


    def loadScaler(self):
        pass



if __name__ == "__main__":

    ## SET SEEDS
    torch.manual_seed(0)
    np.random.seed(0)

    model_config = "Wide-Long"
    FC = NN_Flip_Classifier(model_config)

    ## LOAD RAW DATA
    df_raw = pd.read_csv(f"crazyflie_projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")
    df_bounds = pd.read_csv(f"crazyflie_projects/Policy_Mapping/NeuralNetwork/dataBounds.csv")
    df_raw = pd.concat([df_raw,df_bounds])

    ## ORGANIZE DATA
    RREV = df_raw["RREV_flip_mean"]
    OF_y = df_raw["OF_y_flip_mean"]
    d_ceil = df_raw["flip_d_mean"]
    y = df_raw["landing_rate_4_leg"]
    y = np.where(y < 0.8,0,1)

    X = np.stack((RREV,OF_y,d_ceil),axis=1)
    FC.createScaler(X)


