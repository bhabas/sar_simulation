## STANDARD IMPORTS
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
from scipy.interpolate import griddata
import plotly.graph_objects as go
import pandas as pd
import os
from datetime import datetime

## PYTORCH IMPROTS
import torch
import torch.nn as nn
import torch.nn.functional as F

## SKLEARN IMPORTS
from sklearn.svm import OneClassSVM
from sklearn.model_selection import train_test_split
from sklearn.metrics import *
from sklearn import preprocessing


## ADD SAR_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(0,BASE_PATH)
from crazyflie_logging.data_analysis.Data_Analysis import DataFile


np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/sar_simulation"

class Policy_Trainer():
    def __init__(self,NN_model,SVM_model,model_initials,learning_rate=0.001):

        self.NN_model = NN_model
        self.SVM_model = SVM_model
        self.model_initials = model_initials
        self.learning_rate = learning_rate # Learning Rate

        self.scaler = preprocessing.StandardScaler()
        self.optimizer = torch.optim.Adam(self.NN_model.parameters(), lr=self.learning_rate)

    def createScaler(self,X):
        """Determine scalers to scale training data to mean = 0 and std = 1 before passing through NN

        Args:
            X (np.float array): training data set
        """        

        ## GENERATE SCALERS
        self.scaler = preprocessing.StandardScaler().fit(X)

        ## SAVE SCALER TO FILE 
        np.savetxt(f"{BASEPATH}/crazyflie_projects/SVL_Policy/Policy_Training/Info/Scaler_{self.model_initials}.csv",
            np.stack((self.scaler.mean_,self.scaler.scale_),axis=1),
            fmt='%.5f',
            delimiter=',',
            comments='',
            header=f'mean,std',
            footer='')

    def loadScaler(self):
        """Load scaler terms from csv file
        """        
        
        arr = np.loadtxt(
            open(f"{BASEPATH}/crazyflie_projects/SVL_Policy/Policy_Training/Info/Scaler_{self.model_initials}.csv", "rb"),
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

    def NN_Predict(self,X):
        X_scaled = torch.FloatTensor(self.scaleData(X))

        with torch.no_grad():
            y_pred = self.NN_model.forward(X_scaled)

        return y_pred

    def save_NN_Params(self,SavePath,FileName):
        f = open(SavePath,'a')
        f.truncate(0) ## Clears contents of file

        date_time = datetime.now().strftime('%m/%d-%H:%M')
        f.write(f"// Filename: {FileName} Time: {date_time}\n")
        f.write("static char NN_Params_Flip[] = {\n")
        
        NN_size = np.array([4]).reshape(-1,1)

        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,NN_size,
                    fmt='"%.0f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{NN_size.shape[0]},"\t"{NN_size.shape[1]},"',
                    footer='"*"\n')

        ## EXTEND SCALER ARRAY DIMENSIONS
        scaler_means = self.scaler.mean_.reshape(-1,1)
        scaler_stds = self.scaler.scale_.reshape(-1,1)
        
        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,scaler_means,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_means.shape[0]},"\t"{scaler_means.shape[1]},"',
                    footer='"*"\n')

        np.savetxt(f,scaler_stds,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_stds.shape[0]},"\t"{scaler_stds.shape[1]},"',
                    footer='"*"\n')

        ## SAVE NN LAYER VALUES
        with torch.no_grad():
            ii = 0
            for name, layer in self.NN_model.named_modules():
                if ii > 0: # Skip initialization layer

                    W = layer.weight.numpy()
                    np.savetxt(f,W,
                        fmt='"%.5f,"',
                        delimiter='\t',
                        comments='',
                        header=f'"{W.shape[0]},"\t"{W.shape[1]},"',
                        footer='"*"\n')


                    b = layer.bias.numpy().reshape(-1,1)
                    np.savetxt(f,b,
                        fmt='"%.5f,"',
                        delimiter='\t',
                        comments='',
                        header=f'"{b.shape[0]},"\t"{b.shape[1]},"',
                        footer='"*"\n')

                ii+=1

        f.write("};")
        f.close()

    def load_NN_Params(self,path):
        """Parse model params from C header file into Pytorch model

        Args:
            path (string): string to C header file
        """        
        
        ## READ PARAMS FROM FILE AS STRING
        f = open(path,"r")
        f.readline() # Metadata line
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
            
            ## EXTRACT NUM OF LAYERS
            if ii == 0:
                num_layers = int(arr[0,0])

            ## EXTRACT SCALER FOR MEAN
            elif ii == 1:
                self.scaler.mean_ = arr[:,0]

            ## EXTRACT SCALER FOR VARIANCE
            elif ii == 2:
                self.scaler.scale_ = arr[:,0]

            ## ADD MATRIX TO ARRAY LIST
            else:
                arr_list.append(arr)

        ## EXTRACT LAYERS FROM PYTORCH MODEL
        layer_list = list(dict(self.NN_model.named_modules()).values())[1:]

        ## FILL MATRICES INTO PYTORCH MODEL
        for ii in range(num_layers*2):
            if ii%2 == 0:
                layer_list[ii//2].weight.data = torch.FloatTensor(arr_list[ii])
            else:
                layer_list[ii//2].bias.data = torch.FloatTensor(arr_list[ii].flatten())

    def train_NN_Model(self,X_train,y_train,X_test,y_test,epochs=500):

        ## CONVERT DATA ARRAYS TO TENSORS
        X_train = torch.FloatTensor(self.scaleData(X_train))
        X_test = torch.FloatTensor(self.scaleData(X_test))

        y_train = torch.FloatTensor(y_train)
        y_test = torch.FloatTensor(y_test)

    
        ## DEFINE TRAINING LOSS
        criterion_train_My = nn.MSELoss()
        losses_train = []

        ## DEFINE VALIDATION LOSS
        criterion_test_My = nn.MSELoss()
        losses_test = []

        for ii in range(epochs):

            ## CALC TRAINING LOSS
            y_pred_train = self.NN_model.forward(X_train)

            loss_train_My = criterion_train_My(y_pred_train,y_train)
            loss = loss_train_My
            losses_train.append(loss_train_My.item())

            ## CALC VALIDATION LOSS
            with torch.no_grad():
                y_pred_test = self.NN_model.forward(X_test)
                loss_test_My = criterion_test_My(y_pred_test,y_test)
                losses_test.append(loss_test_My.item())

            if ii%10 == 1:
                print(f"epoch {ii} and loss is: {losses_train[-1]}")

            ## BACKPROPAGATION
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()


        ## PLOT LOSSES AND ACCURACIES
        fig = plt.figure(1,figsize=(6,4))
        ax = fig.add_subplot()
        ax.plot(losses_train,label="Training Loss")
        ax.plot(losses_test,label="Validation Loss")
        ax.set_ylabel("Loss")
        ax.set_title("My Prediction Loss")
        ax.set_yscale('log')
        ax.legend()
        ax.grid()


        plt.tight_layout()
        plt.show()


    def OC_SVM_Predict(self,X):

        X = self.scaleData(X)

        support_vecs = self.SVM_model.support_vectors_
        dual_coeffs = self.SVM_model.dual_coef_
        gamma = self.SVM_model.gamma
        intercept = self.SVM_model.intercept_

        # https://scikit-learn.org/stable/modules/svm.html
        # Eq: decision_val = sum(dual_coeff[ii]*K(supp_vec[ii],X)) + b

        ## PASS STATE VALUE THROUGH OC_SVM
        val = 0
        for ii in range(len(support_vecs)):
            val += dual_coeffs[0,ii]*np.exp(-gamma*np.sum((X-support_vecs[ii])**2))
        val += intercept

        # ## CHECK IF MANUAL CALC DOESN'T MATCH PACKAGE
        # if not np.isclose(val[0],self.SVM_model.decision_function(X))[0]:
        #     raise Exception

        print(f"Func Val: {self.SVM_model.decision_function(X)[0]:.4f} || Custom Val: {val[0]:.4f}")
        # self.SVM_model.decision_function(X)
        return val

    def train_OC_SVM(self,X_train):

        ## SCALE TRAINING DATA
        X_train = self.scaleData(X_train)

        ## FIT OC_SVM MODEL
        self.SVM_model.fit(X_train)

    def save_SVM_Params(self,SavePath,FileName):

        f = open(SavePath,'a')
        f.truncate(0) ## Clears contents of file

        date_time = datetime.now().strftime('%m/%d-%H:%M')
        f.write(f"// Filename: {FileName} Time: {date_time}\n")
        f.write("static char SVM_Params[] = {\n")
        

        ## EXTEND SCALER ARRAY DIMENSIONS
        scaler_means = self.scaler.mean_.reshape(-1,1)
        scaler_stds = self.scaler.scale_.reshape(-1,1)
        
        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,scaler_means,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_means.shape[0]},"\t"{scaler_means.shape[1]},"',
                    footer='"*"\n')

        np.savetxt(f,scaler_stds,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_stds.shape[0]},"\t"{scaler_stds.shape[1]},"',
                    footer='"*"\n')

        gamma = np.array(self.SVM_model._gamma).reshape(-1,1)
        intercept = self.SVM_model._intercept_.reshape(-1,1)
        dual_coeffs = self.SVM_model._dual_coef_.reshape(-1,1)
        support_vecs = self.SVM_model.support_vectors_
        

        ## SAVE GAMMA
        np.savetxt(f,gamma,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{gamma.shape[0]},"\t"{gamma.shape[1]},"',
                    footer='"*"\n')

        ## SAVE INTERCEPT
        np.savetxt(f,intercept,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{intercept.shape[0]},"\t"{intercept.shape[1]},"',
                    footer='"*"\n')

        
        ## SAVE DUAL COEFFS
        np.savetxt(f,dual_coeffs,
                    fmt='"%.4f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{dual_coeffs.shape[0]},"\t"{dual_coeffs.shape[1]},"',
                    footer='"*"\n')

        ## SAVE SUPPORT VECTORS
        np.savetxt(f,support_vecs,
                    fmt='"%.4f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{support_vecs.shape[0]},"\t"{support_vecs.shape[1]},"',
                    footer='"*"\n')

        

        

        f.write("};")
        f.close()

    def load_SVM_Params(self,path):
        """Parse model params from C header file into SVM model

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
            
            ## EXTRACT GAMMA
            elif ii == 2:
                self.SVM_model.gamma = arr[0,0]
            
            ## EXTRACT INTERCEPT
            elif ii == 3:
                self.SVM_model.intercept_ = arr[:,0]

            ## EXTRACT DUAL COEFFS
            elif ii == 4:
                self.SVM_model.dual_coef_ = arr.reshape(1,-1)

            ## EXTRACT SUPPORT VECTORS
            elif ii == 5:
                self.SVM_model.support_vectors_ = arr

        

    def evalModel(self,X,y,LR_bound=0.9):
        """Evaluate the model

        Args:
            X (_type_): Evaluation dataset
            y (_type_): Evaluation dataset
        """        

        ## PREDICT VALUES FROM EVALUATION DATASET
        y_pred = self.NN_Predict(X)
        y_pred = np.where(y_pred[:,0].detach().numpy() < LR_bound,0,1)
        y_class = np.where(y[:,0] < LR_bound,0,1)

        cfnMatrix = confusion_matrix(y_class,y_pred,normalize=None)
        print("\n=========== Model Evaluation ===========")
        print(f"Balanced Accuracy: {balanced_accuracy_score(y_class,y_pred):.3f}")
        print(f"Confusion Matrix: \n{cfnMatrix}")
        print(f"False Positives: {cfnMatrix[0,1]}")
        print(f"False Negatives: {cfnMatrix[1,0]}")
        print(f"Classification Report: {classification_report(y_class,y_pred)}")

    def plotPolicyRegion(self,df,PlotBoundry=True,PlotTraj=(None,0,0),iso_level=0.0):

        fig = go.Figure()

        df["LR_4Leg"] = np.where(df["LR_4Leg"] >= 0.8, 1.0, 0)

        # PLOT DATA POINTS
        fig.add_trace(
            go.Scatter3d(
                ## DATA
                x=df["OFy_flip_mean"],
                y=df["Tau_flip_mean"],
                z=df["D_ceil_flip_mean"],

                ## HOVER DATA
                customdata=df,
                hovertemplate=" \
                    <b>LR: %{customdata[3]:.3f} | My: %{customdata[14]:.3f}</b> \
                    <br>Vel: %{customdata[0]:.2f} | Phi: %{customdata[1]:.2f}</br> \
                    <br>Tau: %{customdata[8]:.3f} | OFy: %{customdata[10]:.3f} </br> \
                    <br>D_ceil: %{customdata[12]:.3f}</br>",

                ## MARKER
                mode='markers',
                marker=dict(
                    size=3,
                    color=df["LR_4Leg"],
                    colorscale='Viridis',   # choose a colorscale
                    cmin=0,
                    cmax=1.6, 
                    opacity=0.8)
            )
        )

        if PlotBoundry:
            ## MESHGRID OF DATA POINTS
            Tau_grid, OF_y_grid, d_ceil_grid = np.meshgrid(
                np.linspace(0.15, 0.35, 60),
                np.linspace(-15, 1, 45),
                np.linspace(0.0, 1.0, 45)
            )

            ## CONCATENATE DATA TO MATCH INPUT SHAPE
            X_grid = np.stack((
                Tau_grid.flatten(),
                OF_y_grid.flatten(),
                d_ceil_grid.flatten()),axis=1)
            
            y_pred_grid = self.SVM_model.decision_function(self.scaleData(X_grid))

            fig.add_trace(
                go.Volume(
                    ## ISO SURFACE
                    x=X_grid[:,1].flatten(),
                    y=X_grid[:,0].flatten(),
                    z=X_grid[:,2].flatten(),
                    value=y_pred_grid.flatten(),
                    
                    surface_count=1,
                    opacity=0.25,
                    isomin=iso_level,
                    isomax=iso_level,
                    colorscale='Viridis',  
                    cmin=0,
                    cmax=0.1,     
                    caps=dict(x_show=False, y_show=False),
                    colorbar=dict(title='Title',) , 
                    hoverinfo='skip'
                )
            )

        if PlotTraj[0] != None:
            dataFile,k_ep,k_run = PlotTraj
            arr = dataFile.grab_stateData(k_ep,k_run,['Tau','OF_y','d_ceil'])
            Tau,OFy,d_ceil = np.split(arr,3,axis=1)
            Tau_trg,OFy_trg,d_ceil_trg,My_trg = dataFile.grab_flip_state(k_ep,k_run,['Tau','OF_y','d_ceil','My'])

            fig.add_trace(
                go.Scatter3d(
                    ## DATA
                    x=OFy.flatten(),
                    y=Tau.flatten(),
                    z=d_ceil.flatten(),

                    ## MARKER
                    mode='lines',
                    marker=dict(
                        color='darkblue',
                        size=0,
                    )
                )
            )

            fig.add_trace(
                go.Scatter3d(
                    ## DATA
                    x=[OFy_tr],
                    y=[Tau_trg],
                    z=[d_ceil_tr],

                    ## HOVER DATA
                    hovertemplate=
                        f"<b>My: {My_tr:.3f} N*mm</b> \
                        <br>Tau: {Tau_trg:.3f} | OFy: {OFy_tr:.3f} </br> \
                        <br>D_ceil: {d_ceil_tr:.3f}</br>",

                    ## MARKER
                    mode='markers',
                    marker=dict(
                        size=3,
                        color='darkblue',
                    )
                )
            )

        

        fig.update_layout(
            scene=dict(
                xaxis_title='OFy [rad/s]',
                yaxis_title='Tau [s]',
                zaxis_title='D_ceiling [m]',
                xaxis_range=[-16,1],
                yaxis_range=[0.4,0.1],
                zaxis_range=[0,1.2],
                xaxis = dict(nticks=4, range=[-15,0],),
            ),

        )
        fig.show()

    def plot_Landing_Rate(self,df,saveFig=False):

        ## COLLECT DATA
        Vel_flip = df.iloc[:]['Vel_flip'].to_numpy()
        Phi_flip = df.iloc[:]['Phi_flip'].to_numpy()
        LR = df.iloc[:]['LR_4Leg'].to_numpy()


        ## GENERATE INTERPOLATION GRID
        Phi_list = np.linspace(20,90,50)
        Vel_list = np.linspace(1.5,4.0,50)        
        Phi_grid,Vel_grid = np.meshgrid(Phi_list,Vel_list)


        ## NORMALIZE DATA BEFORE INTERPOLATION (ALWAYS NORMALIZE!!!)
        def normalize(data,xmin,xmax):
            data = data.astype(np.float64)
            return (data-xmin)/(xmax-xmin)

        Phi_min,Phi_max = Phi_flip.min(),Phi_flip.max()
        Vel_min,Vel_max = Vel_flip.min(),Vel_flip.max()

        Phi_norm = normalize(Phi_flip,Phi_min,Phi_max)
        Vel_norm = normalize(Vel_flip,Vel_min,Vel_max)

        Phi_grid_norm = normalize(Phi_grid,Phi_min,Phi_max)
        Vel_grid_norm = normalize(Vel_grid,Vel_min,Vel_max)
        

        ## INTERPOLATE VALUES
        interp_grid = (Phi_grid_norm, Vel_grid_norm)    # Interpolation grid
        points = np.array((Phi_norm,Vel_norm)).T        # Known data points
        values = LR                                     # Known data values
        LR_interp = griddata(points, values, interp_grid, method='linear',fill_value=0.0)


        ## PLOT DATA
        fig = plt.figure(figsize=(4,4))

        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=1)

        ax = fig.add_subplot(projection='polar')
        ax.contourf(np.radians(Phi_grid),Vel_grid,LR_interp,levels=30,cmap=cmap,norm=norm)
        # ax.scatter(np.radians(Phi_grid.flatten()),Vel_grid.flatten(),c=LR_interp.flatten(),cmap=cmap,norm=norm)
        # ax.scatter(np.radians(Phi_flip),Vel_flip,c=LR,cmap=cmap,norm=norm)

        ax.set_thetamin(30)
        ax.set_thetamax(90)
        ax.set_rmin(0)
        ax.set_rmax(3.5)

        ax.set_xticks(np.radians([30,45,60,75,90]))
        ax.set_yticks([0,1.0,2.0,3.0,3.5])
        plt.show()
 
        if saveFig==True:
            plt.savefig(f'{self.model_initials}_Polar_LR.pdf',dpi=300)


## DEFINE NN MODEL
class NN_Model(nn.Module):
    def __init__(self,in_features=3,h=10,out_features=1):
        super().__init__()
        self.fc1 = nn.Linear(in_features,h)     # Layer 1
        self.fc2 = nn.Linear(h,h)               # Layer 2
        self.fc3 = nn.Linear(h,h)               # Layer 3
        self.out = nn.Linear(h,out_features)    # Layer 4

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.elu(self.fc1(x))
        x = F.elu(self.fc2(x))
        x = F.elu(self.fc3(x))
        x = self.out(x)

        return x



if __name__ == "__main__":

    ## SET SEEDS
    torch.manual_seed(0)
    np.random.seed(0)



    ## DESIGNATE FILE PATHS
    FileName = "NL_LR_Trials.csv"
    # FileName = "NL_SVL_LR_Trials.csv"
    # FileName = "DeepRL_NL_LR.csv"
    model_initials = FileName[:2]
    NN_Param_Path = f'{BASEPATH}/crazyflie_projects/SVL_Policy/Policy_Training/Info/NN_Layers_{model_initials}.h'
    SVM_Param_Path = f'{BASEPATH}/crazyflie_projects/SVL_Policy/Policy_Training/Info/SVM_Params_{model_initials}.h'

    FilePath = f"{BASEPATH}/crazyflie_projects/SVL_Policy/Data_Logs/"
    

    ## PRE-INITIALIZE MODELS
    NN_model = NN_Model()
    SVM_model = OneClassSVM(nu=0.1,gamma=2.0)
    # nu: Controls size of enclosed volume (smaller -> larger volume)
    # gamma: Controls how much the volume matches shape of data (smaller -> less definition)
    #           Increases number of support vectors making calculations longer?
    Policy = Policy_Trainer(NN_model,SVM_model,model_initials)

    ## LOAD DATA
    df_raw = pd.read_csv(FilePath+FileName).dropna() # Collected data
    df_train = df_raw.query("LR_4Leg >= 0.8")

    ## MAX LANDING RATE DATAFRAME
    idx = df_raw.groupby(['Vel_d','Phi_d'])['LR_4Leg'].transform(max) == df_raw['LR_4Leg']
    df_max = df_raw[idx].reset_index()

    ## ORGANIZE DATA
    Tau = df_train["Tau_flip_mean"]
    OF_y = df_train["OFy_flip_mean"]
    d_ceil = df_train["D_ceil_flip_mean"]
    My = df_train["My_mean"]

    ## CREATE SCALER
    X = np.stack((Tau,OF_y,d_ceil),axis=1)
    Policy.createScaler(X)

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING SETS
    data_array = np.stack((Tau,OF_y,d_ceil,My),axis=1)
    df = pd.DataFrame(data_array,columns=['Tau','OFy','d_ceil','My_mean'])

    train_df, test_df = train_test_split(df,test_size=0.10,random_state=73)
    X_train = train_df[['Tau','OFy','d_ceil']].to_numpy()
    y_train = train_df[['My_mean']].to_numpy()

    X_test = test_df[['Tau','OFy','d_ceil']].to_numpy()
    y_test = test_df[['My_mean']].to_numpy()

    


    ## TRAIN NEURAL NETWORK My POLICY
    # Policy.train_NN_Model(X_train,y_train,X_test,y_test,epochs=3200)
    # Policy.save_NN_Params(NN_Param_Path,FileName)
    # Policy.load_NN_Params(NN_Param_Path)
    # print(Policy.NN_Predict(np.array([[0.233,-2.778,0.518]])))

    ## TRAIN OC_SVM FLIP_CLASSIFICATION POLICY
    Policy.train_OC_SVM(X)
    Policy.save_SVM_Params(SVM_Param_Path,FileName)
    print(Policy.OC_SVM_Predict(np.array([[0.233,-2.778,0.518]])))


    Policy.plotPolicyRegion(df_train,PlotBoundry=True,iso_level=0.0)
    # Policy.plotPolicyRegion(df_raw,PlotBoundry=True,iso_level=0.0)
    # Policy.plotPolicyRegion(df_raw,PlotBoundry=False,iso_level=0.0)
    # Policy.plot_Landing_Rate(df_max,saveFig=False)
    
    EXP_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging_exp'))

    # dataPath = f"{EXP_PATH}/crazyflie_logging_exp/local_logs/"
    # dataPath = f"{BASE_PATH}/crazyflie_projects/SVL_Policy/Data_Logs/SVL_Experimental_Data/"

    # fileName = "SVL--NL_3.00_45.00_10-27_11:06" + ".csv"
    # trial = DataFile(dataPath,fileName,dataType='EXP')
    # k_ep = 0
    # Policy.plotPolicyRegion(df_train,PlotBoundry=True,iso_level=0.00,PlotTraj=(trial,k_ep,0))


