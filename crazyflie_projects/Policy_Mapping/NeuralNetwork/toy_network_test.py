import numpy as np


def sigmoid(x):
    z = 1/(1 + np.exp(-x))

    return z

W = np.array([[-0.1548,  0.2339, -0.2190]])
b = np.array([-0.5661]).reshape(-1,1)

X = np.array([1,2,3]).reshape(-1,1)


a = np.dot(W,X)+b

a = sigmoid(a)
print(a)

