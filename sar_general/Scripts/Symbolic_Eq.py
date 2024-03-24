from sympy import symbols, cos, sin, Matrix, pprint, simplify
from sympy import *

# Define the angle symbols
phi, theta, beta_1, beta_2, gamma = symbols('phi theta beta_1 beta_2 gamma')

# Define the first rotation matrix (for example, a rotation around the z-axis)
R_P_W = Matrix([
    [ cos(theta), sin(theta)],
    [-sin(theta), cos(theta)],
])

R_B_W = Matrix([
    [ cos(phi), sin(phi)],
    [-sin(phi), cos(phi)],
])

print("Phi_rel Rotation Matrix:")
pprint(simplify(R_B_W.T*R_P_W))
print("\n\n\n")


## BETA 1 CONVERSION
R_C1_B = Matrix([
    [ sin(gamma), cos(gamma)],
    [-cos(gamma), sin(gamma)],
])


## BETA 1 CONVERSION
R_C1_P = Matrix([
    [ cos(beta_1), sin(beta_1)],
    [-sin(beta_1), cos(beta_1)],
])


print("Beta1 from Phi")
Result = R_C1_B.T*R_B_W.T*R_P_W
pprint(simplify(Result))
print("\n\n\n")

print("Phi from Beta1")
Result = R_C1_B*R_C1_P.T*R_P_W.T
pprint(simplify(Result))
print("\n\n\n")



## BETA 2 CONVERSION

R_C2_B = Matrix([
    [-sin(gamma), cos(gamma)],
    [-cos(gamma),-sin(gamma)],
])

R_C2_P = Matrix([
    [ cos(beta_2), sin(beta_2)],
    [-sin(beta_2), cos(beta_2)],
])

print("Beta2 from Phi")
Result = R_C2_B.T*R_B_W.T*R_P_W
pprint(simplify(Result))
print("\n\n\n")


print("Phi from Beta2")
Result = R_C2_B*R_C2_P.T*R_P_W.T
pprint(simplify(Result))
print("\n\n\n")

