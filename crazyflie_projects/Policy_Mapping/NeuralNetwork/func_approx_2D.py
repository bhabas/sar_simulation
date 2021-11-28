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
def my2dfun(x1, x2,function_select):
    functions = {
        1: np.power(x1,2)-np.power(x2,2), # quadratic function
        2: np.sin(np.sqrt(x1**2 + x2**2)) # sinus
    }
    return functions.get(function_select)


## DEFINE NN MODEL
class Model(nn.Module):
    def __init__(self,in_features=2,h1=8,h2=8,out_features=1):
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
function_select = 1

x1 = np.linspace(-3.0, 3.0, num=batch_size)
x2 = np.linspace(-3.0, 3.0, num=batch_size)

X1,X2 = np.meshgrid(x1,x2)

X = np.stack((X1.flatten(),X2.flatten()),axis=1)



## GENERATE LABELS
y = my2dfun(X1,X2,function_select).reshape(-1,1)



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
# X_eval = np.linspace(-3,10,batch_size).reshape(-1,1)
# X_eval = torch.FloatTensor(X_eval)
with torch.no_grad():
    y_eval = model.forward(X_test)

with torch.no_grad():
    y_eval2 = model.forward(torch.FloatTensor(np.array((2,2))))
print(y_eval2)

## PLOT NETWORK OUTPUTS
fig = plt.figure(1, figsize=(12,6))

ax1 = fig.add_subplot(2,3,1,projection='3d')
ax1.plot_trisurf(X[::5,0],X[::5,1],y[::5].flatten(),cmap='viridis', linewidth=0.2, antialiased=True)
ax1.set_xlabel('x1')
ax1.set_ylabel('x2')
ax1.set_title('Function')

ax2 = fig.add_subplot(2,3,2,projection='3d')
ax2.plot_trisurf(X_test[:,0],X_test[:,1],y_eval.flatten(),cmap='viridis', linewidth=0.2, antialiased=True)
ax2.set_xlabel('x1')
ax2.set_ylabel('x2')
ax2.set_title('Prediction')



fig.tight_layout()
plt.show()