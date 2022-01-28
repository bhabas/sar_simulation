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
        x = torch.sigmoid(self.out(x))

        return x


class NN_Flip_Classifier():

    def __init__(self,model_config):
        self.model_config = model_config

        




    
    
    def train_NN(self,epochs,X_train,y_train,X_test,y_test):
        X_train = torch.FloatTensor(self.scaleData(X_train))
        X_test = torch.FloatTensor(self.scaleData(X_test))

        y_train = torch.FloatTensor(y_train)
        y_test = torch.FloatTensor(y_test)


        model = NN_Model()
        optimizer = torch.optim.Adam(model.parameters(), lr=0.01)

        class_weight = [0.1, 0.5] # Weights as binary classes [0,1]
        
        ## DEFINE TRAINING LOSS
        weights = np.where(y_train==1,class_weight[1],class_weight[0])      # Convert class weights to element weights
        criterion = nn.BCELoss(weight=torch.FloatTensor(weights))       
        losses_train = []
        accuracies_train = []

        ## DEFINE VALIDATION LOSS
        weights_val = np.where(y_test==1,class_weight[1],class_weight[0])   # Convert class weights to element weights
        criterion_val = nn.BCELoss(weight=torch.FloatTensor(weights_val))   
        losses_test = []
        accuracies_test = []


        for ii in range(epochs):

            ## MODEL PREDICTION
            y_pred_train = model.forward(X_train)

            ## CALC TRAINING LOSS
            loss_train = criterion(y_pred_train,y_train)
            losses_train.append(loss_train.item())

            ## CALC TRAINING ACCURACY
            pred_cutoff = 0.5 
            y_pred_train_class = np.where(y_pred_train.detach().numpy() < pred_cutoff,0,1)
            accuracy_train = balanced_accuracy_score(y_train[:,0],y_pred_train_class)
            accuracies_train.append(accuracy_train)

            
            ## CALC VALIDATION LOSS
            with torch.no_grad():
                y_pred_test = model.forward(X_test)

            loss_test = criterion_val(y_pred_test,y_test)
            losses_test.append(loss_test.item())

            ## CALC TESTING ACCURACY
            y_pred_test_class = np.where(y_pred_test.detach().numpy() < pred_cutoff,0,1)
            # accuracy = balanced_accuracy_score(y_test[0],y_pred_test_class[0])
            # accuracies_test.append(accuracy)

            if ii%10 == 1:
                print(f"epoch {ii} and loss is: {loss_train}")

            ## BACKPROPAGATION
            optimizer.zero_grad()
            loss_train.backward()
            optimizer.step()

            
        torch.save(model,f'{BASEPATH}/Pickle_Files/Flip_Network.pt')

        ## PLOT LOSSES AND ACCURACIES
        fig = plt.figure(1,figsize=(12,8))
        ax1 = fig.add_subplot(2,1,1)
        ax1.plot(losses_train,label="Training Loss")
        ax1.plot(losses_test,label="Validation Loss")
        ax1.set_ylabel("Loss")
        ax1.set_title("Losses")
        ax1.set_ylim(0)
        ax1.legend()
        ax1.grid()

        ax2 = fig.add_subplot(2,1,2)
        ax2.plot(accuracies_train,label="Training Accuracry")
        ax2.plot(accuracies_test,label="Validation Accuracy")
        ax2.set_ylim(0,1.1)
        ax2.set_ylabel("Classification Accuracy")
        ax2.set_title("Balanced Accuracy")
        ax2.grid()
        ax2.legend(loc="lower right")


        plt.tight_layout()
        plt.show()

        return model
            

    def generateScaler(self,df):
        
        X = np.stack((
            df["RREV"],
            df["OF_y"],
            df["d_ceil"]),axis=1)
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

        X = scaler.inverse_transform(X_scaled)

        return X
    
    def splitData(self,df):
        train_df, test_df = train_test_split(df,test_size=0.25,random_state=73,shuffle=False)

        ## CONVERT DATA INTO TENSORS
        X_train = train_df[['RREV','OF_y','d_ceil']].to_numpy()
        X_test = test_df[['RREV','OF_y','d_ceil']].to_numpy()

        y_train = train_df[['y']].to_numpy()
        y_test = test_df[['y']].to_numpy()

        return X_train,y_train,X_test,y_test

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
    FC = NN_Flip_Classifier(model_config)

    ## COLLECT AND CLEAN DATA
    df_raw = pd.read_csv(f"crazyflie_projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")
    df_bounds = pd.read_csv(f"crazyflie_projects/Policy_Mapping/NeuralNetwork/dataBounds.csv")
    df = pd.concat([df_raw,df_bounds])

    RREV = df["RREV_flip_mean"]
    OF_y = df["OF_y_flip_mean"]
    d_ceil = df["flip_d_mean"]
    y = df["landing_rate_4_leg"].to_numpy().reshape(-1,1)
    y = np.where(y < 0.8,0,1)

    X = np.stack((RREV,OF_y,d_ceil),axis=1)
    data_array = np.hstack((X,y))
    df = pd.DataFrame(data_array,columns=['RREV','OF_y','d_ceil','y'])
    df = df.sort_values(by='y')

    ## UPDATE DATA SCALER
    FC.generateScaler(df)

    ## SPLIT DATA
    X_train,y_train,X_test,y_test = FC.splitData(df)
    model = FC.train_NN(500,X_train,y_train,X_test,y_test)
    # model = torch.load(f'{BASEPATH}/Pickle_Files/Flip_Network.pt')
    # FC.testAcc(model,X,y)
    # FC.passData(model,X)


