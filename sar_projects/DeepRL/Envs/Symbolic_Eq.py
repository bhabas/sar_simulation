from sympy import symbols, cos, sin, Matrix, pprint, simplify
from sympy import *

# Define the angle symbols
phi, theta, beta_1, beta_2, gamma = symbols('phi theta beta_1 beta_2 gamma')

# Define the first rotation matrix (for example, a rotation around the z-axis)
R_W_P = Matrix([
    [ cos(theta), sin(theta)],
    [-sin(theta), cos(theta)],
])

R_W_B = Matrix([
    [ cos(phi), sin(phi)],
    [-sin(phi), cos(phi)],
])

# pprint(simplify(R_W_B.T*R_W_P))
# print()

## BETA 1 CONVERSION

R_B_Beta1 = Matrix([
    [ sin(gamma), cos(gamma)],
    [-cos(gamma), sin(gamma)],
])

R_P_Beta1 = Matrix([
    [ cos(beta_1), sin(beta_1)],
    [-sin(beta_1), cos(beta_1)],
])

print("Phi from Beta1")
Result = R_B_Beta1*R_P_Beta1.T*R_W_P.T
pprint(simplify(Result))
print()
print()


print("Beta1 from Phi")
Result = R_B_Beta1.T*R_W_B.T*R_W_P 
pprint(simplify(Result))
print()
print()

# test = Matrix([
#     [ sin(gamma - phi + theta + pi),-cos(gamma - phi + theta + pi)],
#     [-cos(gamma - phi + theta + pi),-sin(gamma - phi + theta + pi)],
# ])
# pprint(simplify(test))
# print()
# print()
# print()



## BETA 2 CONVERSION

R_B_Beta2 = Matrix([
    [-sin(gamma), cos(gamma)],
    [-cos(gamma),-sin(gamma)],
])

R_P_Beta2 = Matrix([
    [ cos(beta_2), sin(beta_2)],
    [-sin(beta_2), cos(beta_2)],
])

print("Phi from Beta2")
Result = R_B_Beta2*R_P_Beta2.T*R_W_P.T
pprint(simplify(Result))
print()
print()


print("Beta2 from Phi")
Result = R_B_Beta2.T*R_W_B.T*R_W_P 
pprint(simplify(Result))
print()
print()

