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
from imblearn.over_sampling import RandomOverSampler

import plotly.graph_objects as go
from NN_Trainer import NN_Trainer


np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping"

## DEFINE NN MODEL
class NN_Flip_Model(nn.Module):
    def __init__(self,in_features=3,h=20,out_features=1):
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



if __name__ == "__main__":

    ## SET SEEDS
    torch.manual_seed(0)
    np.random.seed(0)

    LR_bound = 0.9 # Classify states with LR higher than this value as valid
    model_initials = "NL_DR"
    model = NN_Flip_Model()
    FlipClassifier = NN_Trainer(model,model_initials,LR_bound)

    ## LOAD DATA
    df_raw = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_DR/NL_LR_Trials_DR.csv").dropna() # Collected data
    df_bounds = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_Raw/Boundary.csv") # Manufactured data to mold policy region
    df_all = pd.concat([df_raw,df_bounds])

    ## ORGANIZE DATA
    Tau = df_all["Tau_flip_mean"]
    OF_y = df_all["OFy_flip_mean"]
    d_ceil = df_all["D_ceil_flip_mean"]
    X = np.stack((Tau,OF_y,d_ceil),axis=1)

    y = df_all["LR_4leg"]

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING SETS
    data_array = np.stack((Tau,OF_y,d_ceil,y),axis=1)
    df = pd.DataFrame(data_array,columns=['Tau','OFy','d_ceil','LR'])
    df = df.sort_values(by='LR')

    train_df, test_df = train_test_split(df,test_size=0.10,random_state=73)
    X_train = train_df[['Tau','OFy','d_ceil']].to_numpy()
    y_train = train_df[['LR']].to_numpy()

    X_test = test_df[['Tau','OFy','d_ceil']].to_numpy()
    y_test = test_df[['LR']].to_numpy()

    # ## OVERSAMPLE TRAINING DATASET BECAUSE THERE'S LESS VALID LR SAMPLES THAN INVALID
    # oversample = RandomOverSampler(sampling_strategy='minority')
    # X_train, y_train = oversample.fit_resample(X_train, y_train)
    # y_train = y_train.reshape(-1,1)

    


    Param_Path = f'{BASEPATH}/NeuralNetwork/Info/NN_Layers_Flip_{model_initials}.h'
    FlipClassifier.createScaler(X)
    FlipClassifier.trainClassifier_Model(X_train,y_train,X_test,y_test,LR_bound=LR_bound,epochs=2000)
    FlipClassifier.saveParams(Param_Path)
    FlipClassifier.evalModel(X,y)
 

    FlipClassifier.loadModelFromParams(Param_Path)
    FlipClassifier.evalModel(X,y,LR_bound=LR_bound)
    FlipClassifier.plotModel(df_raw,LR_bound=LR_bound)



