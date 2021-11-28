import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split
import os


import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F


## DESIGNATE FUNCTION USED
def myfun(x,function_select):
    functions = {
        1: np.power(x,2),   # quadratic function
        2: np.sin(x),       # sin
        3: np.sign(x),      # signum
        4: np.exp(x),       # exponential function
        5: (np.power(x,3)+ np.power(x-10,4))/np.power(x-20,2),  # polynomial
        6: 1+np.power(x,2)/4000-np.cos(x)                       # Griewank function
        # for Griewank, we need more than 50, does not generalize well (we must make sure not to have more parameters than samples in order to avoid overfitting)
    }

    return functions.get(function_select)

## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=1,h1=8,h2=8,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h1) # Fully connected layer
        self.fc2 = nn.Linear(h1,h2)
        self.out = nn.Linear(h2,out_features)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.tanh(self.fc1(x))
        x = F.tanh(self.fc2(x))
        x = self.out(x)

        return x



## GENERATE TRAINING DATA
batch_size = 50
function_select = 2

X = np.linspace(-1,7,batch_size).reshape(-1,1)
y = myfun(X,function_select)



## SPLIT DATA FEATURES INTO TRAINING DATA & TESTING DATA
X_train, X_test, y_train, y_test = train_test_split(X,y,test_size=0.2,random_state=33)


## CONVERT DATA INTO TENSORS
X_train = torch.FloatTensor(X_train)
X_test = torch.FloatTensor(X_test)

y_train = torch.FloatTensor(y_train)
y_test = torch.FloatTensor(y_test)


## INITIALIZE NEURAL NETWORK MODEL
torch.manual_seed(32)
model = Model()

## DEFINE OPTIMIZER AND LOSS FUNCTION (CRITERION)
criterion = nn.MSELoss(reduction='mean')
optimizer = torch.optim.Adam(model.parameters(),lr=0.01) # Model parameters are the layers of model


epochs = 10000
losses = []

for i in range(epochs):

    ## MODEL PREDICTION
    y_pred = model.forward(X_train)

    ## CALCULATE LOSS/ERROR
    loss = criterion(y_pred,y_train)
    losses.append(loss)

    if i%10 == 1:
        print(f"epoch {i} and loss is: {loss}")

    

    ## BACKPROPAGATION
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()



# ## PLOT LOSS VS EPOCHS
# plt.plot(range(epochs),losses)
# plt.ylabel('Loss')
# plt.xlabel('Epoch')



## DEFINE EVALUATION RANGE
X_eval = np.linspace(-3,10,batch_size).reshape(-1,1)
X_eval = torch.FloatTensor(X_eval)
with torch.no_grad():
    y_eval = model.forward(X_eval)

## PLOT NETWORK OUTPUTS
fig = plt.figure(1, figsize=(12,6))
ax1 = fig.add_subplot(1,2,1)
ax1.axvspan(X.flatten()[0], X.flatten()[-1], alpha=0.15, color='limegreen')
ax1.plot(X_eval, myfun(X_eval,function_select), '-', color='royalblue', linewidth=1.0)
ax1.plot(X_eval, y_eval, '-', label='output', color='darkorange', linewidth=2.0)
ax1.plot(X_train, myfun(X_train,function_select), '.', color='royalblue')

ax1.grid()
ax1.set_xlabel('x')
ax1.set_ylabel('f(x)')
ax1.set_title('%d neurons in hidden layer with %d epochs of training' % (8,epochs))
ax1.legend(['Function: f(x)', 'Neural Network: g(x)', 'Training Set'])

ax2 = fig.add_subplot(1,2,2)
ax2.axvspan(X.flatten()[0], X.flatten()[-1], alpha=0.15, color='limegreen')
ax2.plot(X_eval, np.abs(y_eval-myfun(X_eval,function_select)), '-', label='output', color='firebrick', linewidth=2.0)

ax2.grid()
ax2.set_xlabel('x')
ax2.set_ylabel('Error')
ax2.set_title('Absolute difference between prediction and actual function')
ax2.legend(['Error |f(x)-g(x)|'])
fig.tight_layout()
plt.show()