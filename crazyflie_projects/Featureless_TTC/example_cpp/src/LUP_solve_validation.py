import numpy as np


A = np.array([
    [1,2,3],
    [4,5,6],
    [7,8,1]])

b = np.array([5,3,1])

x = np.linalg.pinv(A)@b

print(x)