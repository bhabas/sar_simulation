#!/usr/bin/env python3

from __future__ import print_function

from crazyflie_msgs.srv import Policy_Values,Policy_ValuesResponse
import rospy
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from sklearn import preprocessing

BASEPATH = "crazyflie_projects/Policy_Mapping/Data_Analysis"

class NN_Flip_Classifier(nn.Module):
    def __init__(self,in_features=2,h1=5,h2=5,out_features=1):
        super().__init__()
        h = 20
        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        # self.fc1 = nn.Linear(3,h) # Fully connected layer
        # self.out = nn.Linear(h,1)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.elu(self.fc1(x))
        x = torch.sigmoid(self.out(x))

        return x

NN_Classifier = torch.load(f'{BASEPATH}/Pickle_Files/Flip_Network.pt')

class NN_Policy_Value(nn.Module):
    def __init__(self,in_features=3,h=10,out_features=1):
        super().__init__()

        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(in_features,h) # Fully connected layer
        self.fc2 = nn.Linear(h,h)
        self.out = nn.Linear(h,out_features)

    def forward(self,x):
 
        # PASS DATA THROUGH NETWORK
        x = torch.sigmoid(self.fc1(x))
        x = torch.sigmoid(self.fc2(x))
        x = self.out(x)

        return x
NN_Policy = torch.load(f'{BASEPATH}/Pickle_Files/Policy_Network.pt')




def handle_policy(req):
    # print("Returning [%s + %s = %s]"%(req.OF_y, req.RREV, (req.a + req.b)))

    # ## EVALUATE NN MODEL
    
    X = torch.FloatTensor([-3.3781,  1.1145, -0.9872])
    X = torch.FloatTensor([req.RREV, req.OF_y, req.d_ceil])
    with torch.no_grad():
        y_pred = NN_Classifier.forward(X)
        y_pred_class = np.where(y_pred.detach().numpy() < 0.5,0,1)
        y_pred_class = bool(y_pred_class)

        y_pred2 = NN_Policy.forward(X)

    print(y_pred,y_pred2)

    return Policy_ValuesResponse(y_pred_class,y_pred2)

def policy_server():
    rospy.init_node('policy_NN_server')
    s = rospy.Service('policy_NN', Policy_Values, handle_policy)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    policy_server()