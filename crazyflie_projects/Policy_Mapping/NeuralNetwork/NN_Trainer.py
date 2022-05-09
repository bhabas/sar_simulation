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

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping"

class NN_Trainer():
    def __init__(self,model,model_initials,LR_bound):

        self.model = model
        self.model_initials = model_initials
        self.LR_bound = LR_bound

        self.scaler = preprocessing.StandardScaler()

    def createScaler(self,X):
        """Determine scalers to scale training data to mean = 0 and std = 1 before passing through NN

        Args:
            X (np.float array): training data set
        """        

        ## GENERATE SCALERS
        self.scaler = preprocessing.StandardScaler().fit(X)

        ## SAVE SCALER TO FILE 
        np.savetxt(f"{BASEPATH}/NeuralNetwork/Info/Scaler_Flip_{self.model_initials}.csv",
            np.stack((self.scaler.mean_,self.scaler.scale_),axis=1),
            fmt='%.6f',
            delimiter=',',
            comments='',
            header=f'mean,std',
            footer='')

    def loadScaler(self):
        """Load scaler terms from csv file
        """        
        
        arr = np.loadtxt(
            open(f"{BASEPATH}/Info/Scaler_Flip_{self.model_initials}.csv", "rb"),
            delimiter=",",
            skiprows=1)

        self.scaler = preprocessing.StandardScaler()
        self.scaler.mean_ = arr[:,0]
        self.scaler.scale_ = arr[:,1]

    def scaleData(self,X):
        """Scale the given data
        """        

        return self.scaler.transform(X)

    def unscaleData(self,X_scaled):
        """Unscale the given data
        """        

        return self.scaler.inverse_transform(X_scaled)

    def modelPredict(self,X):
        X_scaled = torch.FloatTensor(self.scaleData(X))

        with torch.no_grad():
            y_pred = self.model.forward(X_scaled)

        return y_pred

    def saveParams(self,path):
        f = open(path,'a')
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

    def loadModelFromParams(self,path):
        """Parse model params from C header file into Pytorch model

        Args:
            path (string): string to C header file
        """        
        
        ## READ PARAMS FROM FILE AS STRING
        f = open(path,"r")
        f.readline() # Skip first line
        file_string = f.read()

        ## CLEAN STRING FROM EXTRA CHARACTERS
        char_remove = ['"','\n','\t']
        for char in char_remove:
            file_string = file_string.replace(char,'')

        ## CREATE LIST OF MATRIX STRINGS
        matrix_list = file_string.split("*")

        ## SPLIT STRINGS INTO PROPER SCALERS AND MATRICES
        arr_list = []
        for ii,matrix_str in enumerate(matrix_list[:-1]):
            val_list = matrix_str[:-1].split(",")   # Break string up into components
            num_rows = int(val_list[0])             # Extract number of rows
            num_cols = int(val_list[1])             # Extract number of columns
            arr = np.array(val_list[2:]).astype(np.float64).reshape(num_rows,num_cols) # Extract matrix

            ## EXTRACT SCALER FOR MEAN
            if ii == 0:
                self.scaler.mean_ = arr[:,0]

            ## EXTRACT SCALER FOR VARIANCE
            elif ii == 1:
                self.scaler.scale_ = arr[:,0]

            ## ADD MATRIX TO ARRAY LIST
            else:
                arr_list.append(arr)

        ## EXTRACT LAYERS FROM PYTORCH MODEL
        layer_list = list(dict(self.model.named_modules()).values())[1:]

        ## FILL MATRICES INTO PYTORCH MODEL
        for ii in range(len(arr_list)):
            if ii%2 == 0:
                layer_list[ii//2].weight.data = torch.FloatTensor(arr_list[ii])
            else:
                layer_list[ii//2].bias.data = torch.FloatTensor(arr_list[ii].flatten())


    def trainClassifier_Model(self,X_train,y_train,X_test,y_test,LR_bound=0.9,epochs=500):

        ## CONVERT DATA ARRAYS TO TENSORS
        X_train = torch.FloatTensor(self.scaleData(X_train))
        X_test = torch.FloatTensor(self.scaleData(X_test))

        y_train = torch.FloatTensor(y_train)
        y_test = torch.FloatTensor(y_test)

        ## INIT MODEL AND OPTIMIZER
        optimizer = torch.optim.Adam(self.model.parameters(), lr=0.001)
    
        ## DEFINE TRAINING LOSS
        criterion_train = nn.MSELoss()       
        losses_train = []
        accuracies_train = []

        ## DEFINE VALIDATION LOSS
        criterion_test = nn.MSELoss()   
        losses_test = []
        accuracies_test = []

        for ii in range(epochs):


            ## CALC TRAINING LOSS
            y_pred_train = self.model.forward(X_train)
            loss_train = criterion_train(y_pred_train,y_train)
            losses_train.append(loss_train.item())

            ## CALC VALIDATION LOSS
            with torch.no_grad():
                y_pred_test = self.model.forward(X_test)
                loss_test = criterion_test(y_pred_test,y_test)
                losses_test.append(loss_test.item())



            ## CALC TRAINING ACCURACY
            y_pred_train_class = np.where(y_pred_train.detach().numpy() < LR_bound,0,1)
            y_train_class = np.where(y_train[:,0] < LR_bound,0,1)
            accuracy_train = balanced_accuracy_score(y_train_class,y_pred_train_class)
            accuracies_train.append(accuracy_train)

            ## CALC TESTING ACCURACY
            y_pred_test_class = np.where(y_pred_test.detach().numpy() < LR_bound,0,1)
            y_test_class = np.where(y_test[:,0] < LR_bound,0,1)
            accuracy_test = balanced_accuracy_score(y_test_class,y_pred_test_class)
            accuracies_test.append(accuracy_test)

            if ii%10 == 1:
                print(f"epoch {ii} and loss is: {loss_train}")

            ## BACKPROPAGATION
            optimizer.zero_grad()
            loss_train.backward()
            optimizer.step()


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

    def evalModel(self,X,y,LR_bound=0.9):
        """Evaluate the model

        Args:
            X (_type_): Evaluation dataset
            y (_type_): Evaluation dataset
        """        

        ## PREDICT VALUES FROM EVALUATION DATASET
        y_pred = self.modelPredict(X)
        y_pred = np.where(y_pred.detach().numpy() < LR_bound,0,1)
        y_class = np.where(y < LR_bound,0,1)

        cfnMatrix = confusion_matrix(y_class,y_pred,normalize=None)
        print("\n=========== Model Evaluation ===========")
        print(f"Balanced Accuracy: {balanced_accuracy_score(y_class,y_pred):.3f}")
        print(f"Confusion Matrix: \n{cfnMatrix}")
        print(f"False Positives: {cfnMatrix[0,1]}")
        print(f"False Negatives: {cfnMatrix[1,0]}")
        print(f"Classification Report: {classification_report(y_class,y_pred)}")

    def plotModel(self,df_custom,LR_bound=0.9):

        ## CREATE GRID
        Tau_grid, OF_y_grid, d_ceil_grid = np.meshgrid(
            np.linspace(0, 0.5, 30),
            np.linspace(-15, 0, 30),
            np.linspace(0, 1.5, 30)
        )

        ## CONCATENATE DATA TO MATCH INPUT
        X_grid = np.stack((
            Tau_grid.flatten(),
            OF_y_grid.flatten(),
            d_ceil_grid.flatten()),axis=1)

        y_pred_grid = self.modelPredict(X_grid)

        fig = go.Figure()

        fig.add_trace(
            go.Volume(
                ## ISO SURFACE
                x=X_grid[:,1].flatten(),
                y=X_grid[:,0].flatten(),
                z=X_grid[:,2].flatten(),
                value=y_pred_grid.flatten(),
                
                surface_count=10,
                opacity=0.1,
                isomin=LR_bound-0.05,
                isomax=LR_bound,
                # colorscale='Viridis',  
                cmin=0,
                cmax=1,     
                caps=dict(x_show=False, y_show=False),
                colorbar=dict(title='Title',) , 
                hoverinfo='skip'
            )
        )

        ## PLOT DATA POINTS
        fig.add_trace(
            go.Scatter3d(
                ## DATA
                x=df_custom["OFy_flip_mean"],
                y=df_custom["Tau_flip_mean"],
                z=df_custom["D_ceil_flip_mean"],

                ## HOVER DATA
                customdata=df_custom,
                hovertemplate=" \
                    <b>LR: %{customdata[3]:.3f}</b> \
                    <br>OFy: %{customdata[10]:.3f} Vel: %{customdata[0]:.2f} </br> \
                    <br>Tau: %{customdata[8]:.3f} Phi: %{customdata[1]:.0f}</br> \
                    <br>D_ceil: %{customdata[12]:.3f}</br>",

                ## MARKER
                mode='markers',
                marker=dict(
                    size=3,
                    color=df_custom["LR_4leg"],                # set color to an array/list of desired values
                    colorscale='Viridis',   # choose a colorscale
                    opacity=1.0)
            )
        )

        fig.update_layout(
            scene=dict(
                xaxis_title='OFy [rad/s]',
                yaxis_title='Tau [s]',
                zaxis_title='D_ceiling [m]',
                xaxis_range=[-20,1],
                yaxis_range=[0.4,0.1],
                zaxis_range=[0,1.2],
            ),
        )
        fig.show()