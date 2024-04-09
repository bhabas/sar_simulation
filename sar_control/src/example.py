import numpy as np


a = 1.0
J = 10.0
S = 0.25
t_j = a/J

A = 1/2*a
B = 1/2*J*t_j**2 + a*t_j
C = 1/2*a*t_j**2 + 1/2*J*t_j**3 - 1/4*S

t_acc = (-B + np.sqrt(B**2 - 4*A*C))/(2*A)
print(t_j)
print(t_acc)