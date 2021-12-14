## STANDARD IMPORTS
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

## SKLEARN IMPORTS
from sklearn import svm
from sklearn import preprocessing
from sklearn.model_selection import train_test_split
from sklearn.datasets import make_blobs
from sklearn.metrics import *
# accuracy_score, balanced_accuracy_score,confusion_matrix,classification_report,roc_curve,roc_auc_score


## PYTORCH IMPORTS
import torch
import torch.nn as nn
import torch.nn.functional as F

## LOCAL IMPORTS
# from NN_Training import *


os.system("clear")
BASEPATH = "crazyflie_projects/Policy_Mapping/ML_Comparison"


## GENERATE DATA
n_samples_1 = 3000
centers = [[0.0,0.0,0.0]]
clusters_std = [3.0]
X, y = make_blobs(
    n_features=3,
    n_samples=[n_samples_1],
    centers=centers,
    cluster_std=clusters_std,
    random_state=0,
    shuffle=False,
)

X1 = X[:,0]
X2 = X[:,1]
X3 = X[:,2]

r = np.sqrt(X1**2 + X2**2 + X3**2)
y = [1 if ii < 2.5 else 0 for ii in r]

## CONVERT DATA TO DATAFRAME
data_array = np.stack((X1,X2,X3,y,r),axis=1)
df = pd.DataFrame(data_array,columns=['X1','X2','X3','y','r'])
df = df.sort_values(by='X1')

## SPLIT DATA FEATURES INTO TRAINING AND TESTING DATA
train_df, test_df = train_test_split(df,test_size=0.2,random_state=34)
test_df = test_df.sort_values(by='r')


## CONVERT DATA INTO TENSORS
X_train = torch.FloatTensor(train_df[['X1','X2','X3']].to_numpy())
X_test = torch.FloatTensor(test_df[['X1','X2','X3']].to_numpy())

y_train = torch.FloatTensor(train_df[['y','r']].to_numpy())
y_test = torch.FloatTensor(test_df[['y','r']].to_numpy())


## TRAIN SVM MODEL
clf = svm.SVC(kernel='rbf',gamma="auto",class_weight={1: 20})
clf.fit(X_train,y_train[:,0])

## EVALUATE SVM MODEL
y_pred_SVM = clf.predict(X_test)
print(balanced_accuracy_score(y_test[:,0],y_pred_SVM))
print(confusion_matrix(y_test[:,0],y_pred_SVM,normalize=None))
print(classification_report(y_test[:,0],y_pred_SVM))

## TRAIN NN MODEL
epochs = 1000
# train_model(epochs,X_train,y_train[:,0])


# ## EVALUATE NN MODEL
# model = torch.load(f'{BASEPATH}/Func_approx_3D.pt')

# with torch.no_grad():
#     y_pred_NN = model.forward(X_test)
#     y_pred_NN = np.round(y_pred_NN,0)
# print(balanced_accuracy_score(y_test[:,0],y_pred_NN))
# print(confusion_matrix(y_test[:,0],y_pred_NN,normalize=None))
# print(classification_report(y_test[:,0],y_pred_NN))




fig = plt.figure(1,figsize=(8,8))

## PLOT MODEL
ax1 = fig.add_subplot(111,projection='3d')
ax1.scatter(
    test_df.query('X1<=0.0')['X1'],
    test_df.query('X1<=0.0')['X2'],
    test_df.query('X1<=0.0')['X3'],
    c = test_df.query('X1<=0.0')['y'],
    cmap='jet',linewidth=0.2,antialiased=True)
ax1.set_xlabel('X1')
ax1.set_ylabel('X2')
ax1.set_ylabel('X3')
ax1.set_title('Function')
ax1.set_xlim(-10,10)
ax1.set_ylim(-10,10)
ax1.set_zlim(-10,10)


plt.show()



