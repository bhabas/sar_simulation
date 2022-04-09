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
        self.loadModel()

        self.scaler = None
        self.loadScaler()

    def modelInitials(self): # RETURNS INITIALS FOR MODEL
        str = self.model_config
        charA = str[0] # [W]ide
        charB = str[str.find("-")+1] # [L]ong

        self.model_initials = charA+charB

    def loadModel(self):
        self.model = torch.load(f'{BASEPATH}/Pickle_Files/NN_Flip_Classifier_{self.model_initials}.pt')

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
        
        arr = np.loadtxt(
            open(f"{BASEPATH}/Info/Scaler_Flip_{self.model_initials}.csv", "rb"),
            delimiter=",",
            skiprows=1)

        self.scaler = preprocessing.StandardScaler()
        self.scaler.mean_ = arr[:,0]
        self.scaler.scale_ = arr[:,1]

    def scaleData(self,X):

        return self.scaler.transform(X)

    def unscaleData(self,X_scaled):

        return self.scaler.inverse_transform(X_scaled)

    def trainModel(self,X_train,y_train,X_test,y_test,epochs=500,saveModel=True):

        ## CONVERT DATA ARRAYS TO TENSORS
        X_train = torch.FloatTensor(self.scaleData(X_train))
        X_test = torch.FloatTensor(self.scaleData(X_test))

        y_train = torch.FloatTensor(y_train)
        y_test = torch.FloatTensor(y_test)

        ## INIT MODEL AND OPTIMIZER
        self.model = NN_Model()
        optimizer = torch.optim.Adam(self.model.parameters(), lr=0.01)
        class_weight = [0.1, 0.5] # Weights as binary classes [0,1]
        pred_cutoff = 0.5 
    
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


            ## CALC TRAINING LOSS
            y_pred_train = self.model.forward(X_train)
            loss_train = criterion(y_pred_train,y_train)
            losses_train.append(loss_train.item())

            ## CALC TRAINING ACCURACY
            y_pred_train_class = np.where(y_pred_train.detach().numpy() < pred_cutoff,0,1)
            accuracy_train = balanced_accuracy_score(y_train[:,0],y_pred_train_class)
            accuracies_train.append(accuracy_train)

            
            ## CALC VALIDATION LOSS
            with torch.no_grad():
                y_pred_test = self.model.forward(X_test)

            loss_test = criterion_val(y_pred_test,y_test)
            losses_test.append(loss_test.item())

            ## CALC TESTING ACCURACY
            y_pred_test_class = np.where(y_pred_test.detach().numpy() < pred_cutoff,0,1)
            accuracy_test = balanced_accuracy_score(y_test[:,0],y_pred_test_class)
            accuracies_test.append(accuracy_test)

            if ii%10 == 1:
                print(f"epoch {ii} and loss is: {loss_train}")

            ## BACKPROPAGATION
            optimizer.zero_grad()
            loss_train.backward()
            optimizer.step()

        if saveModel:
            torch.save(self.model,f'{BASEPATH}/Pickle_Files/NN_Flip_Classifier_{self.model_initials}.pt')

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

    def modelForward(self,X):
        X_scaled = torch.FloatTensor(self.scaleData(X))

        with torch.no_grad():
            y_pred = self.model.forward(X_scaled)

        return y_pred

    def evalModel(self,X,y):

        with torch.no_grad():
            y_pred = self.modelForward(X)
            y_pred = np.where(y_pred.detach().numpy() < 0.5,0,1)

        cfnMatrix = confusion_matrix(y,y_pred,normalize=None)
        print("\n=========== Model Evaluation ===========")
        print(f"Balanced Accuracy: {balanced_accuracy_score(y,y_pred):.3f}")
        print(f"Confusion Matrix: \n{cfnMatrix}")
        print(f"False Negatives: {cfnMatrix[0,1]}")
        print(f"False Positives: {cfnMatrix[1,0]}")
        print(f"Classification Report: {classification_report(y,y_pred)}")


    def plotModel(self,df_custom):

        ## CREATE GRID
        RREV_grid, OF_y_grid, d_ceil_grid = np.meshgrid(
            np.linspace(0, 8, 30),
            np.linspace(-15, 0, 30),
            np.linspace(0, 1.5, 30)
        )

        ## CONCATENATE DATA TO MATCH INPUT
        X_grid = np.stack((
            RREV_grid.flatten(),
            OF_y_grid.flatten(),
            d_ceil_grid.flatten()),axis=1)

        y_pred_grid = self.modelForward(X_grid)

        fig = go.Figure()

        fig.add_trace(
            go.Volume(
            # go.Isosurface(
                x=X_grid[:,1].flatten(),
                y=X_grid[:,0].flatten(),
                z=X_grid[:,2].flatten(),
                value=y_pred_grid.flatten(),
                surface_count=10,
                opacity=0.1,
                isomin=0.85,
                isomax=0.90,
                # colorscale='Viridis',  
                cmin=0,
                cmax=1,     
                caps=dict(x_show=False, y_show=False),
                colorbar=dict(title='Title',) , 
            ))

        fig.add_trace(
        go.Scatter3d(
            x=df_custom["OF_y_flip_mean"],
            y=df_custom["RREV_flip_mean"],
            z=df_custom["flip_d_mean"],
            mode='markers',
            marker=dict(
                size=3,
                color=df_custom["landing_rate_4_leg"],                # set color to an array/list of desired values
                colorscale='Viridis',   # choose a colorscale
                opacity=1.0)
        ))

        fig.update_layout(
            title='something', 
            autosize=False,
            width=750, 
            height=750,
            margin=dict(l=65, r=50, b=65, t=90),
            scene=dict(
                xaxis_title='OFy [rad/s]',
                yaxis_title='RREV [rad/s]',
                zaxis_title='D_ceiling [m]',
            ),
        )
        fig.show()

    def saveParams(self):
        f = open(f'{BASEPATH}/Info/NN_Layers_Flip_{self.model_initials}.h','a')
        f.truncate(0) ## Clears contents of file
        f.write("static char NN_Params_Flip[] = {\n")
        

        ## EXTEND SCALER ARRAY DIMENSIONS
        scaler_means = self.scaler.mean_.reshape(-1,1)
        scaler_stds = self.scaler.scale_.reshape(-1,1)
        
        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,scaler_means,
                    fmt='"%.6f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_means.shape[0]},"\t"{scaler_means.shape[1]},"',
                    footer='"*"\n')

        np.savetxt(f,scaler_stds,
                    fmt='"%.6f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_stds.shape[0]},"\t"{scaler_stds.shape[1]},"',
                    footer='"*"\n')

        ## SAVE NN LAYER VALUES
        with torch.no_grad():
            ii = 0
            for name, layer in self.model.named_modules():
                if ii > 0: # Skip initialization layer

                    W = layer.weight.numpy()
                    np.savetxt(f,W,
                        fmt='"%.6f,"',
                        delimiter='\t',
                        comments='',
                        header=f'"{W.shape[0]},"\t"{W.shape[1]},"',
                        footer='"*"\n')


                    b = layer.bias.numpy().reshape(-1,1)
                    np.savetxt(f,b,
                        fmt='"%.6f,"',
                        delimiter='\t',
                        comments='',
                        header=f'"{b.shape[0]},"\t"{b.shape[1]},"',
                        footer='"*"\n')

                ii+=1

        f.write("};")
        f.close()

    def loadModelFromParams(self):
        path = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping/NeuralNetwork/Info/NN_Layers_Flip_WL.h"

        f = open(path,"r")
        f.readline()
        file_string = f.read()
        char_remove = ['"','\n','\t']
        for char in char_remove:
            file_string = file_string.replace(char,'')

        matrix_list = file_string.split("*")

        named_layers = dict(self.model.named_modules())
        layer_list = list(dict(self.model.named_modules()).values())[1:]
        arr_list = []

        for ii,matrix_str in enumerate(matrix_list[:-1]):
            val_list = matrix_str[:-1].split(",")
            num_rows = int(val_list[0])
            num_cols = int(val_list[1])
            arr = np.array(val_list[2:]).astype(np.float64).reshape(num_rows,num_cols)

            if ii == 0:
                self.scaler.mean_ = arr[:,0]

            elif ii == 1:
                self.scaler.scale_ = arr[:,0]

            else:
                arr_list.append(arr)
                # with torch.no_grad():
                #     list(named_layers.values())[ii-1].weight.data = torch.FloatTensor(arr)

        for ii in range(len(arr_list)):
            if ii%2 == 0:
                layer_list[ii//2].weight.data = torch.FloatTensor(arr_list[ii])
            else:
                layer_list[ii//2].bias.data = torch.FloatTensor(arr_list[ii].flatten())
            


        #     self.scaler.mean_ = arr[:,0]
        # self.scaler.scale_ = arr[:,1]
            


if __name__ == "__main__":

    ## SET SEEDS
    torch.manual_seed(0)
    np.random.seed(0)

    model_config = "Wide-Long"
    FC = NN_Flip_Classifier(model_config)

    ## LOAD RAW DATA
    df_raw = pd.read_csv(f"crazyflie_projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")
    df_bounds = pd.read_csv(f"crazyflie_projects/Policy_Mapping/NeuralNetwork/dataBounds.csv")
    # df_raw = pd.concat([df_raw,df_bounds])

    ## ORGANIZE DATA
    RREV = df_raw["RREV_flip_mean"]
    OF_y = df_raw["OF_y_flip_mean"]
    d_ceil = df_raw["flip_d_mean"]
    y = df_raw["landing_rate_4_leg"]
    y = np.where(y < 0.8,0,1)

    X = np.stack((RREV,OF_y,d_ceil),axis=1)

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING SETS
    data_array = np.stack((RREV,OF_y,d_ceil,y),axis=1)
    df = pd.DataFrame(data_array,columns=['RREV','OF_y','d_ceil','y'])
    # df = df.sort_values(by='y')

    train_df, test_df = train_test_split(df,test_size=0.25,random_state=73)
    test_df = test_df.sort_values(by='y')

    X_train = train_df[['RREV','OF_y','d_ceil']].to_numpy()
    X_test = test_df[['RREV','OF_y','d_ceil']].to_numpy()
    y_train = train_df[['y']].to_numpy()
    y_test = test_df[['y']].to_numpy()



    # FC.createScaler(X)
    # FC.trainModel(X_train,y_train,X_test,y_test,epochs=400)
    # FC.saveParams()
    # FC.evalModel(X,y)


    FC.loadModelFromParams()
    FC.evalModel(X,y)
    FC.plotModel(df_raw)

    pass


