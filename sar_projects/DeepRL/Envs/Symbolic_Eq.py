from sympy import symbols, cos, sin, Matrix, pprint, simplify
from sympy import *

# Define the angle symbols
phi, theta, beta_1, beta_2, gamma = symbols('phi theta beta_1 beta_2 gamma')

# Define the first rotation matrix (for example, a rotation around the z-axis)
R_BW = Matrix([
    [ cos(phi), sin(phi)],
    [-sin(phi), cos(phi)],
])

R_PW = Matrix([
    [-cos(theta), sin(theta)],
    [-sin(theta),-cos(theta)],
])

## BETA 1 CONVERSION

R_Beta1P = Matrix([
    [-cos(beta_1), sin(beta_1)],
    [-sin(beta_1),-cos(beta_1)],
])

R_Beta1B = Matrix([
    [ -sin(gamma),-cos(gamma)],
    [  cos(gamma),-sin(gamma)],
])

Result = R_PW*R_Beta1P*R_Beta1B.T
pprint(simplify(Result))
print()

## BETA 2 CONVERSION

R_Beta2P = Matrix([
    [-sin(beta_2),-cos(beta_2)],
    [ cos(beta_2),-sin(beta_2)],
])

R_Beta2B = Matrix([
    [ sin(gamma),-cos(gamma)],
    [ cos(gamma), sin(gamma)],
])

Result = R_PW*R_Beta2P*R_Beta2B.T
pprint(simplify(Result))
