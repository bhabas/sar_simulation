## STANDARD IMPORTS
import numpy as np
import pandas as pd
import plotly.graph_objs as go
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

## PYTORCH IMPORTS
import torch
import torch.nn as nn
import torch.nn.functional as F

## SKLEARN IMPORTS
from sklearn.model_selection import train_test_split
from sklearn.metrics import *
from sklearn import preprocessing

np.set_printoptions(suppress=True)
BASEPATH = "crazyflie_projects/Policy_Mapping/NeuralNetwork"

## DEFINE NN MODEL
class NN_Model(nn.Module):
    def __init__(self,h=10):
        super().__init__()

        ## INIT LAYERS
        self.fc1 = nn.Linear(3,h) # Layer 1
        self.fc2 = nn.Linear(h,h) # Layer 2
        self.out = nn.Linear(h,1) # Layer 3

    def forward(self,x):
        
        ## PASS DATA THROUGH NETWORK
        x = F.elu(self.fc1(x))
        x = F.elu(self.fc2(x))
        x = F.sigmoid(self.out(x))

        return x


class NN_Flip_Classifier():

    def __init__(self,model_config):
        self.model_config = model_config

        




    
    
    def train_NN(self,X_train,y_train,X_test,y_test):
        X_train_scaled = FC.scaleData(X_train)
        X_test_scaled = FC.scaleData(X_test)


        model = NN_Model()
        optimizer = torch.optim.Adam(model.parameters(), lr=0.01)

    def generateScaler(self,df):
        
        X = np.stack((
            df["RREV_flip_mean"],
            df["OF_y_flip_mean"],
            df["flip_d_mean"]),axis=1)
        scaler = preprocessing.StandardScaler().fit(X)


        df_scale = pd.DataFrame(
            np.vstack((scaler.mean_,scaler.scale_)).T,
            columns=['mean','std'])
        df_scale.to_csv(f"{BASEPATH}/Info/Scaler_Flip_{self.model_config}.csv",index=False,float_format="%.3f")


    def scaleData(self,X):
        scaler_df = pd.read_csv(f"{BASEPATH}/Info/Scaler_Flip_{self.model_config}.csv")
        scaler = preprocessing.StandardScaler()
        scaler.mean_ = scaler_df['mean'].to_numpy()
        scaler.scale_ = scaler_df['std'].to_numpy()
        X_scaled = scaler.transform(X)

        return X_scaled

    def unscaleData(self,X_scaled):
        scaler_df = pd.read_csv(f"{BASEPATH}/Info/Scaler_Flip_{self.model_config}.csv")
        
        scaler = preprocessing.StandardScaler()
        scaler.mean_ = scaler_df['mean'].to_numpy()
        scaler.scale_ = scaler_df['std'].to_numpy()

        X_scaled = scaler.inverse_transform(X_scaled)

        return X
    
    def splitData(self):
        pass

    def testAcc(self):
        pass

    def passData(self):
        pass

    def saveParams(self):
        pass

    def loadParams(self):
        pass


    

if __name__ == '__main__':

    torch.manual_seed(0)
    np.random.seed(0)

    model_config = "Wide-Long"
    df_raw = pd.read_csv(f"crazyflie_projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")
    df_bounds = pd.read_csv(f"crazyflie_projects/Policy_Mapping/NeuralNetwork/dataBounds.csv")
    df = pd.concat([df_raw,df_bounds])

    FC = NN_Flip_Classifier(model_config)
    FC.generateScaler(df)
    X = np.array([[-10,5,1.2]])
    X_scaled = FC.scaleData(X)
    print(X_scaled)
    print(FC.unscaleData(X_scaled))
    # X_train,y_train,X_test,y_test = FC.splitData(df)
    # model = FC.train_NN(500,X_train,y_train,X_test,y_test)
    # model = torch.load(f'{BASEPATH}/Pickle_Files/Flip_Network.pt')
    # FC.testAcc(model,X,y)
    # FC.passData(model,X)


