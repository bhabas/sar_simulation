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

class Model(nn.Module):
    def __init__(self,in_features=2,h1=5,h2=5,out_features=1):
        super().__init__()
        h = 20
        # Input Layer (4 features) --> h1 (N) --> h2 (N) --> output (3 classes)
        self.fc1 = nn.Linear(3,h) # Fully connected layer
        self.out = nn.Linear(h,1)

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.elu(self.fc1(x))
        x = torch.sigmoid(self.out(x))

        return x

model = torch.load(f'{BASEPATH}/Pickle_Files/Classifier_3D.pt')


def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))

    # ## EVALUATE NN MODEL
    
    X = torch.FloatTensor([-3.3781,  1.1145, -0.9872])
    X = torch.FloatTensor([req.a, req.b, req.c])
    with torch.no_grad():
        y_pred = model.forward(X)
        y_pred_class = np.where(y_pred.detach().numpy() < 0.5,0,1)

    print(y_pred)

    return Policy_ValuesResponse(y_pred.item())

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', Policy_Values, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()