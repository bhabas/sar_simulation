## STANDARD IMPORTS
from importlib_metadata import requires
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

## PYTORCH IMPROTS
import torch
import torch.nn as nn
import torch.nn.functional as F

## SKLEARN IMPORTS
from sklearn.model_selection import train_test_split
from sklearn.datasets import *
from sklearn.metrics import *
from sklearn import preprocessing

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping/NeuralNetwork"


## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=2,h1=5,h2=5,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h1) # Fully connected layer
        self.fc2 = nn.Linear(h1,h2)
        self.out = nn.Linear(h2,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = torch.sigmoid(self.fc1(x))
        x = torch.sigmoid(self.fc2(x))
        x = torch.sigmoid(self.out(x))

        return x

def train_model(model,X_train,y_train,X_test,y_test,epochs=500):

    
    

    class_weight = [1, 5] # Weights as binary classes [0,1]

    y_train_class = y_train
    weights_train = np.where(y_train_class==1,class_weight[1],class_weight[0]) # Classify and weight values

    y_test_class = y_test
    weights_test = np.where(y_test_class==1,class_weight[1],class_weight[0])


    ## DEFINE TRAINING LOSS
    criterion_train_class = nn.BCELoss(weight=torch.FloatTensor(weights_train))
    losses_train = []
    accuracies_train = []


    ## DEFINE VALIDATION LOSS
    criterion_test_class = nn.BCELoss(weight=torch.FloatTensor(weights_test))
    losses_test = []
    accuracies_test = []

    optimizer = torch.optim.Adam(model.parameters(),lr=0.01) #  Model parameters are the layers of model

    for ii in range(epochs):


        ## CALC TRAINING LOSS
        y_pred_train = model.forward(X_train)

        


        ## INITIATE POINTS
        x = np.linspace(-1,3,20)
        y = np.linspace(-1,3,20)
        XX,YY = np.meshgrid(x,y)
        vals = np.stack((XX.flatten(),YY.flatten()),axis=1)
        X_vol = torch.FloatTensor(vals)

        y_pred_vol = model.forward(X_vol)


        ## FIND OPTIMAL THRESHOLD VALUE
        precision,recall,thresholds = precision_recall_curve(y_test,model.forward(X_test).detach().numpy())
        fscore = (2*precision*recall)/(precision+recall)
        idx = np.argmax(fscore)
        threshold = thresholds[idx]


        ## CALC NUMBER OF POINTS IN OPTIMAL THRESHOLD
        loss_vol = np.sum(np.where(y_pred_vol>=threshold,1,0))/500



        loss_train_class = criterion_train_class(y_pred_train,y_train_class)
        loss = loss_train_class
        losses_train.append(loss.item())

        ## CALC VALIDATION LOSS
        with torch.no_grad():
            y_pred_test = model.forward(X_test)
            loss_test_class = criterion_test_class(y_pred_test,y_test_class)
            losses_test.append(loss_test_class.item())




        # ## CALC TRAINING ACCURACY
        # y_pred_train_class = np.where(y_pred_train[:,0].detach().numpy() > LR_bound,1,0)
        # accuracy_train = balanced_accuracy_score(np.where(y_train[:,0] > LR_bound,1,0),y_pred_train_class)
        # accuracies_train.append(accuracy_train)

        # ## CALC TESTING ACCURACY
        # y_pred_test_class = np.where(y_pred_test[:,0].detach().numpy() < LR_bound,0,1)
        # accuracy_test = balanced_accuracy_score(np.where(y_test[:,0] < LR_bound,0,1),y_pred_test_class)
        # accuracies_test.append(accuracy_test)

        if ii%10 == 1:
            print(f"epoch {ii} and loss is: {losses_train[-1]}")

        ## BACKPROPAGATION
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()


    ## PLOT LOSSES AND ACCURACIES
    fig = plt.figure(1,figsize=(12,8))
    ax1 = fig.add_subplot(2,1,1)
    ax1.plot(losses_train,label="Training Loss")
    ax1.plot(losses_test,label="Validation Loss")
    ax1.set_ylabel("Loss")
    ax1.set_title("Losses")
    # ax1.set_ylim(0)
    ax1.set_yscale('log')
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

    torch.save(model,f'{BASEPATH}/Classifier_2D.pt')


if __name__ == '__main__':

    torch.manual_seed(22)
    

    model = Model()

    ## GENERATE DATA
    X, y = make_classification(
            n_samples=10_000, 
            n_features=2, 
            n_redundant=0,
            n_clusters_per_class=1, 
            weights=[0.98], 
            flip_y=0, 
            random_state=2,
            class_sep=1.1)

    X1 = X[:,0]
    X2 = X[:,1]


    theta = np.linspace(0,2*np.pi,5000)
    radius = np.ones_like(theta)*6

    X1_bound = radius*np.cos(theta)
    X2_bound = radius*np.sin(theta)

    X1 = np.hstack((X1,X1_bound))
    X2 = np.hstack((X2,X2_bound))
    y = np.hstack((y,np.zeros_like(X1_bound)))

    ## CONVERT DATA TO PANDAS DATAFRAME
    data_array = np.stack((X1,X2,y),axis=1)
    df = pd.DataFrame(data_array,columns=['X1','X2','y'])
    df = df.sort_values(by='y')


    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
    train_df, test_df = train_test_split(df,test_size=0.25,random_state=33)
    test_df = test_df.sort_values(by='y')


    ## CONVERT DATA INTO TENSORS
    X_train = train_df[['X1','X2']].to_numpy()
    X_test = test_df[['X1','X2']].to_numpy()

    y_train = train_df[['y']].to_numpy()
    y_test = test_df[['y']].to_numpy()

    ## CONVERT DATA ARRAYS TO TENSORS
    X_train = torch.FloatTensor(X_train)
    X_test = torch.FloatTensor(X_test)

    y_train = torch.FloatTensor(y_train)
    y_test = torch.FloatTensor(y_test)
    




    ## TRAIN NN MODEL
    train_model(model,X_train,y_train,X_test,y_test,epochs=1500)

    ## EVALUATE NN MODEL
    model = torch.load(f'{BASEPATH}/Classifier_2D.pt')

    ## PLOT PRECISION-RECALL CURVE
    with torch.no_grad():
        y_pred = model.forward(X_test)

    ## FIND OPTIMAL THRESHOLD
    precision,recall,thresholds = precision_recall_curve(y_test,y_pred)
    fscore = (2*precision*recall)/(precision+recall)
    idx = np.argmax(fscore)
    print(f'Best Threshold: {thresholds[idx]:.3f} \tF-Score: {fscore[idx]:.3f}')

    

    ## FIND OPTIMAL THRESHOLD
    def to_labels(pos_probs, threshold):
        return (pos_probs >= threshold).astype('int')

    threshold_arr = np.arange(0, 1, 0.001)
    scores = [f1_score(y_test, to_labels(y_pred.numpy(), threshold)) for threshold in threshold_arr]
    ix = np.argmax(scores)
    print(f'Best Threshold: {threshold_arr[ix]:.3f} \tF-Score: {scores[ix]:.3f}')


    ## PLOT PRECISION-RECALL CURVE
    no_skill = len(y_test[y_test==1])/len(y_test)

    fig = plt.figure()
    ax = fig.add_subplot(111)

    ax.plot([0,1],[no_skill,no_skill],linestyle='--',label='No Skill')
    ax.plot(recall,precision,marker='.',label='NN')
    ax.scatter(recall[idx],precision[idx],marker='o',color='black',label='Optimal Threshold')

    ax.set_xlabel('Recall')
    ax.set_ylabel('Precision')
    ax.legend()

    plt.show()

    with torch.no_grad():
        x1 = np.linspace(-5,5,100)
        x2 = np.linspace(-5,5,100)

        X1_test,X2_test = np.meshgrid(x1,x2)
        X_contour = np.stack((X1_test.flatten(),X2_test.flatten()),axis=1)
        X_contour = torch.FloatTensor(X_contour)


        y_contour = model.forward(X_contour).reshape(100,100)
        # contours = measure.find_contours(y_contour.numpy(), 0.75)

        
    x = np.linspace(-1,3,20)
    y = np.linspace(-1,3,20)
    XX,YY = np.meshgrid(x,y)
    vals = np.stack((XX.flatten(),YY.flatten()),axis=1)




    ## PLOT NETWORK OUTPUTS
    fig = plt.figure(1,figsize=(12,6))

    ax1 = fig.add_subplot(1,2,1)

    ax1.contour(X1_test,X2_test,y_contour,levels=[thresholds[idx]],cmap='jet')
    ax1.contour(X1_test,X2_test,y_contour,levels=[threshold_arr[ix]],cmap='jet')

    ax1.scatter(test_df['X1'],test_df['X2'],c=test_df['y'],cmap='jet',linewidth=0.2,antialiased=True)
    # ax1.scatter(vals[:,0],vals[:,1],alpha=0.5)



    

    ax1.grid()
    ax1.set_xlabel('X1')
    ax1.set_ylabel('X2')
    ax1.legend()
  

    fig.tight_layout()
    plt.show()
